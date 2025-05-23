/*
 * ex11_master_master.cpp
 *
 *  Created on: Feb 22, 2010
 *      Author: Christopher Dellin
 *      Author: Dan Cody
 *      Author: Brian Zenowich
 *  	Modified on: Sep 2024
 * 		Author: Faezeh Haghverd
 * 		Author: Amir Noohian
 */

#include <iostream>
#include <string>
#include <vector>
#include <sys/stat.h> // For mkdir()
#include <sys/types.h> // For mode_t
#include <boost/thread.hpp>
#include <cstdio>
#include <cstring>
#include <cerrno>

#include <barrett/os.h>
#include <barrett/detail/stl_utils.h>
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>
#include <barrett/log.h>

#define BARRETT_SMF_VALIDATE_ARGS
#include <barrett/standard_main_function.h>
#include <Dynamics.hpp>
#include "ex11_master_master.h"


using namespace barrett;
using detail::waitForEnter;


bool validate_args(int argc, char** argv) {
	if (argc != 2  &&  argc != 3) {
		printf("Usage: %s <remoteHost> [--auto]\n", argv[0]);
		printf("  --auto : Automatically link WAMs and start Hand or Gimbals Hand Controller thread\n");

		return false;
	}

	return true;
}

//Split function
std::vector<std::string> split(const std::string& str, char delimiter) {
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(str);

    // Manual splitting
    while (std::getline(tokenStream, token)) {
        size_t pos = 0;
        std::string item;
        while ((pos = token.find(delimiter)) != std::string::npos) {
            item = token.substr(0, pos);
            tokens.push_back(item);
            token.erase(0, pos + 1);
        }
        // Add the last token
        tokens.push_back(token);
    }
    return tokens;
}

// Merged function to get an Eigen::VectorXd from an environment variable
template <size_t DOF>
math::Matrix<DOF, 1, double> getEnvEigenVector(const std::string& varName, const math::Matrix<DOF, 1, double>& defaultValue) {
    const char* envVar = std::getenv(varName.c_str());
    if (envVar) {
        std::vector<std::string> stringElements = split(envVar, ',');

        // If the size of the input doesn't match DOF, return the default value
        if (stringElements.size() != DOF) {
            std::cerr << "Vector size mismatch for " << varName << std::endl;
            return defaultValue;
        }

        math::Matrix<DOF, 1, double> matrixVec;
        for (size_t i = 0; i < stringElements.size(); ++i) {
            std::istringstream iss(stringElements[i]);
            double value;
            if (iss >> value) {
                matrixVec(i, 0) = value;  // Assuming math::Matrix supports (row, col) access
            } else {
                std::cerr << "Invalid value in vector for " << varName << std::endl;
                return defaultValue;
            }
        }
        return matrixVec;
    }
    return defaultValue;
}

// Function to get a double parameter from an environment variable or return a default value
double getEnvDouble(const std::string& varName, double defaultValue) {
    const char* envVar = std::getenv(varName.c_str());
    if (envVar) {
        std::istringstream iss(envVar);
        double value;
        if (iss >> value) {
            return value;
        } else {
            std::cerr << "Invalid double value for " << varName << std::endl;
        }
    }
    return defaultValue;
}

template <size_t DOF>
typename units::JointVelocities<DOF>::type saturateJv(
    const typename units::JointVelocities<DOF>::type& x,
    const typename units::JointVelocities<DOF>::type& limit) {
    int index;
    double minRatio;

    minRatio = (limit.cwise() / (x.cwise().abs())).minCoeff(&index);
    if (minRatio < 1.0) {
        return minRatio * x;
    } else {
        return x;
    }
}

template <size_t DOF>
typename units::JointAccelerations<DOF>::type saturateJa(
    const typename units::JointAccelerations<DOF>::type& x,
    const typename units::JointAccelerations<DOF>::type& limit) {
    int index;
    double minRatio;

    minRatio = (limit.cwise() / (x.cwise().abs())).minCoeff(&index);
    if (minRatio < 1.0) {
        return minRatio * x;
    } else {
        return x;
    }
}

template <size_t DOF>
typename units::JointTorques<DOF>::type saturateJt(
    const typename units::JointTorques<DOF>::type& x,
    const typename units::JointTorques<DOF>::type& limit) {
    int index;
    double minRatio;

    minRatio = (limit.cwise() / (x.cwise().abs())).minCoeff(&index);
    if (minRatio < 1.0) {
        return minRatio * x;
    } else {
        return x;
    }
}

//string config_path = "/home/robot/catkin_ws/src/barrett-ros-pkg_zeusV/wam_bringup/launch/with_hand_config_teleop_gains/default.conf";
template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam) {

	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
	const jp_type HOME_POS = wam.getJointPositions();
	wam.gravityCompensate();

	char tmpFile_kinematics[] = "/tmp/btXXXXXX";
	if (mkstemp(tmpFile_kinematics) == -1) {
		printf("ERROR: Couldn't create temporary file: %s\n", strerror(errno));
		return 1;
	}

	char tmpFile_dynamics[] = "/tmp/btXXXXXX";
	if (mkstemp(tmpFile_dynamics) == -1) {
		printf("ERROR: Couldn't create temporary file: %s\n", strerror(errno));
		return 1;
	}

	//Definning syn pos
	jp_type SYNC_POS_default; // the position each WAM should move to before linking
    SYNC_POS_default[1] = -1.5;
    SYNC_POS_default[2] = -0.01;
    SYNC_POS_default[3] = 3.11;
	jp_type SYNC_POS = jp_type(getEnvEigenVector<DOF>("SYNC_POS", v_type(SYNC_POS_default)));

	//Master Master System
	MasterMaster<DOF> mm(pm.getExecutionManager(), argv[1]);
	systems::connect(wam.jpOutput, mm.input);

	//FeedFWD
	double h_omega_p_default = 25.0;
	double h_omega_p = getEnvDouble("h_omega", h_omega_p_default);
	systems::FirstOrderFilter<jp_type> hp1;
	hp1.setHighPass(jp_type(h_omega_p), jp_type(h_omega_p));
	systems::FirstOrderFilter<jp_type> hp2;
	hp2.setHighPass(jp_type(h_omega_p), jp_type(h_omega_p));
	systems::Gain<jp_type, double, jv_type> jvDes(1.0);
	systems::Gain<jp_type, double, ja_type> jaDes(1.0);
	jv_type jvLimits_default;
	jvLimits_default << 0.1, 2, 2, 2, 0.1, 0.1, 0.1;
	jv_type jvLimits = jv_type(getEnvEigenVector<DOF>("jv_limits", v_type(jvLimits_default)));
	systems::Callback<jv_type> jvSat(boost::bind(saturateJv<DOF>,_1, jvLimits));
	ja_type jaLimits_default;
	jaLimits_default << 0.1, 2.0, 2.0, 2.0, 0.1, 0.1, 0.1;
	ja_type jaLimits = ja_type(getEnvEigenVector<DOF>("ja_limits", v_type(jaLimits_default)));
	systems::Callback<ja_type> jaSat(boost::bind(saturateJa<DOF>,_1, jaLimits));
	jt_type jtLimits_default;
	jtLimits_default << 0.1, 20, 20, 15, 0.1, 0.1, 0.1;
	jt_type jtLimits = jt_type(getEnvEigenVector<DOF>("jtLimits", v_type(jtLimits_default)));
	systems::Callback<jt_type> feedSat(boost::bind(saturateJt<DOF>,_1, jtLimits));
	systems::Ramp time(pm.getExecutionManager(), 1.0);
	Dynamics<DOF> feedFWD;

	connect(mm.output, hp1.input);
	connect(hp1.output, hp2.input);
	connect(hp2.output, jaDes.input);
	connect(hp1.output, jvDes.input);
	connect(jvDes.output, jvSat.input);
	connect(jaDes.output, jaSat.input);
	pm.getExecutionManager()->startManaging(hp2);
	sleep(1);

	connect(mm.output, feedFWD.jpInputDynamics);
	connect(jvSat.output, feedFWD.jvInputDynamics);
    connect(jaSat.output, feedFWD.jaInputDynamics);

	connect(feedFWD.dynamicsFeedFWD, feedSat.input);

	//ID for arm dynamics
	Dynamics<DOF> inverseDyn;
	systems::FirstOrderFilter<jp_type> hp3;
	systems::FirstOrderFilter<jp_type> hp4;
	hp3.setHighPass(jp_type(h_omega_p), jp_type(h_omega_p));
	hp4.setHighPass(jp_type(h_omega_p), jp_type(h_omega_p));
	systems::Gain<jp_type, double, ja_type> jaCur(1.0);

	connect(wam.jpOutput, hp3.input);
	connect(hp3.output, hp4.input);
	connect(hp4.output, jaCur.input);
	pm.getExecutionManager()->startManaging(hp4);
	sleep(1);
	connect(wam.jpOutput, inverseDyn.jpInputDynamics);
	connect(wam.jvOutput, inverseDyn.jvInputDynamics);
    connect(jaCur.output, inverseDyn.jaInputDynamics);

	//	RT Logging stuff : config
	systems::Ramp timelog(pm.getExecutionManager(), 1.0);
	systems::TupleGrouper<double, jp_type, jp_type, jv_type, jv_type, ja_type, ja_type> tg_kinematics;
	systems::connect(timelog.output, tg_kinematics.template getInput<0>());
	systems::connect(mm.output, tg_kinematics.template getInput<1>());
	systems::connect(wam.jpOutput, tg_kinematics.template getInput<2>());
	systems::connect(jvDes.output, tg_kinematics.template getInput<3>());
	systems::connect(wam.jvOutput, tg_kinematics.template getInput<4>());
	systems::connect(jaDes.output, tg_kinematics.template getInput<5>());
	systems::connect(jaCur.output, tg_kinematics.template getInput<6>());

	typedef boost::tuple<double, jp_type, jp_type, jv_type, jv_type, ja_type, ja_type> tuple_type_kinematics;
	const size_t PERIOD_MULTIPLIER = 1;
	systems::PeriodicDataLogger<tuple_type_kinematics> logger_kinematics(
			pm.getExecutionManager(),
			new log::RealTimeWriter<tuple_type_kinematics>(tmpFile_kinematics, PERIOD_MULTIPLIER * pm.getExecutionManager()->getPeriod()),
			PERIOD_MULTIPLIER);

	//	RT Logging stuff : jt_types
	systems::TupleGrouper<double, jt_type, jt_type, jt_type, jt_type, jt_type> tg_dynamics;
	systems::connect(timelog.output, tg_dynamics.template getInput<0>());
	systems::connect(wam.jtSum.output, tg_dynamics.template getInput<1>());
	systems::connect(wam.gravity.output, tg_dynamics.template getInput<2>());
	systems::connect(feedFWD.dynamicsFeedFWD, tg_dynamics.template getInput<3>());
	systems::connect(inverseDyn.dynamicsFeedFWD, tg_dynamics.template getInput<4>());
	systems::connect(wam.jpController.controlOutput, tg_dynamics.template getInput<5>());

	typedef boost::tuple<double, jt_type, jt_type, jt_type, jt_type, jt_type> tuple_type_dynamics;
	systems::PeriodicDataLogger<tuple_type_dynamics> logger_dynamics(
			pm.getExecutionManager(),
			new log::RealTimeWriter<tuple_type_dynamics>(tmpFile_dynamics, PERIOD_MULTIPLIER * pm.getExecutionManager()->getPeriod()),
			PERIOD_MULTIPLIER);

	std::vector<std::string> autoCmds;
	std::string line;
	v_type gainTmp;
    bool exit_called = false;
    while (!exit_called) {
		if (autoCmds.empty()) {
			printf(">>> ");
			std::getline(std::cin, line);
		} else {
			line = autoCmds.back();
			autoCmds.pop_back();
		}

		switch (line[0]) {
		case 'l':
			if (mm.isLinked()) {
				mm.unlink();
			} else {
				wam.moveTo(SYNC_POS);

				printf("Press [Enter] to link with the other WAM.");
				waitForEnter();
				mm.tryLink();
				wam.trackReferenceSignal(mm.output);
				connect(feedSat.output, wam.input);

				btsleep(0.1);  // wait an execution cycle or two
				if (mm.isLinked()) {
					printf("Linked with remote WAM.\n");
				} else {
					printf("WARNING: Linking was unsuccessful.\n");
				}
			}

			break;

		case 't':{
			size_t jointIndex;
			{
				size_t jointNumber;
				std::cout << "\tJoint: ";
				std::cin >> jointNumber;
				jointIndex = jointNumber - 1;

				if (jointIndex >= DOF) {
					std::cout << "\tBad joint number: " << jointNumber;
					break;
				}
			}

			char gainId;
			std::cout << "\tGain identifier (p, i, or d): ";
			std::cin >> line;
			gainId = line[0];

			std::cout << "\tCurrent value: ";
			switch (gainId) {
			case 'p':
				gainTmp = wam.jpController.getKp();
				break;
			case 'i':
				gainTmp = wam.jpController.getKi();
				break;
			case 'd':
				gainTmp = wam.jpController.getKd();
				break;

			default:
				std::cout << "\tBad gain identifier.";
			}
			std::cout << gainTmp[jointIndex] << std::endl;

			std::cout << "\tNew value: ";
			std::cin >> gainTmp[jointIndex];
			switch (gainId) {
			case 'p':
				wam.jpController.setKp(gainTmp);
				break;
			case 'i':
				wam.jpController.setKi(gainTmp);
				break;
			case 'd':
				wam.jpController.setKd(gainTmp);
                break;

			default:
				std::cout << "\tBad gain identifier.";
			}

			break;
		}

		case 'c':{
			timelog.start();
			connect(tg_kinematics.output, logger_kinematics.input);
			connect(tg_dynamics.output, logger_dynamics.input);
			printf("Logging started.\n");
			break;
		}

		case 's':{
				logger_kinematics.closeLog();
				logger_dynamics.closeLog();
				printf("Logging stopped.\n");
				timelog.stop();
				timelog.reset();
				break;
		}
        case 'e':{
                 exit_called = true;
                 break;
        }

        default:
			printf("\n");
			printf("    'l' to toggle linking with other WAM\n");
			printf("    't' to tune control gains\n");
			printf("    'c' to start collect data\n");
			printf("    's' to save data\n");
			break;

	}

	}


    // Create the directory using mkdir()
	std::string folderName = argv[2];
	std::string fullPath = ".data/" + folderName;
    if (mkdir(fullPath.c_str(), 0777) == -1) {
        std::cerr << "Error: Could not create directory. It might already exist." << std::endl;
        return 1; // Exit the program if directory creation fails
    }

	std::string kinematicsFilename = fullPath + "/kinematics.txt";
	std::string dynamicsFilename = fullPath + "/dynamics.txt";
	std::string configFilename = fullPath + "/config.txt";
	std::ofstream configFile(configFilename.c_str());

	//Config File Writing
	configFile << "Master Master Teleop with Dynamic Compensation-Follower.\n";
	configFile << "Kinematics data: time, desired joint pos, feedback joint pos, desired joint vel, feedback joint vel, desired joint acc, feedback joint acc\n";
	configFile << "Dynamics data: time, wam joint torque input, wam gravity input, dynamic feed forward, inverse dynamic, PD\n";
	configFile << "Joint Position PID Controller: \nkp: " << wam.jpController.getKp() << "\nki: " << wam.jpController.getKi()<<  "\nkd: "<< wam.jpController.getKd() <<"\nControl Signal Limit: " << wam.jpController.getControlSignalLimit() <<".\n";
	configFile << "Sync Pos:" << SYNC_POS;
	configFile << "\nDesired Joint Vel Saturation Limit: " << jvLimits;
	configFile << "\nDesired Joint Acc Saturation Limit: " << jaLimits;
	configFile << "\nCurrent Joint Acc Saturation Limit: " << jaLimits;
	configFile << "\nFeedFwd Torque Saturation Limit: " << jtLimits;
	configFile << "\nHigh Pass Filter Frq:" << h_omega_p;
	// configFile << "\nHigh Pass Filter Frq used to get current acc:" << h_omega_p;

	log::Reader<tuple_type_kinematics> lr_kinematics(tmpFile_kinematics);
	lr_kinematics.exportCSV(kinematicsFilename.c_str());
	log::Reader<tuple_type_dynamics> lr_Dynamics(tmpFile_dynamics);
	lr_Dynamics.exportCSV(dynamicsFilename.c_str());
	configFile.close();
	printf("Output written to %s folder.\n", fullPath.c_str());

	unlink(tmpFile_kinematics);
	unlink(tmpFile_dynamics);

	return 0;
}


