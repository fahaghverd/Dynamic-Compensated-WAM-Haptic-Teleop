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


template<size_t DOF>
class sinextorq: public systems::System {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

// IO
public:
	Input<double> timef;
	Output<jt_type> extorq;

protected:
	typename System::Output<jt_type>::Value* extorqValue;

public:
	explicit sinextorq(v_type A, double f, const std::string& sysName = "sinextorq"):
		System(sysName), timef(this), extorq(this, &extorqValue), A(A), f(f){}

	virtual ~sinextorq() { this->mandatoryCleanUp(); }

protected:
    double f, t;
    v_type A;
	jt_type Extorq;

	virtual void operate() {
        t = this->timef.getValue();
        Extorq = A * sin(2 * M_PI * f* t);

		this->extorqValue->setData(&Extorq);
	}

private:
	DISALLOW_COPY_AND_ASSIGN(sinextorq);
};

//string config_path = "/home/robot/catkin_ws/src/barrett-ros-pkg_zeusV/wam_bringup/launch/with_hand_config_teleop_gains/default.conf";
template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam) {

	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
	const jp_type HOME_POS = wam.getJointPositions();
	wam.gravityCompensate();

	char tmpFile_kinematics[] = "/tmp/btXXXXXX";
	if (mkstemp(tmpFile_kinematics) == -1) {
		printf("ERROR: Couldn't create temporary file!\n");
		return 1;
	}

	char tmpFile_dynamics[] = "/tmp/btXXXXXX";
	if (mkstemp(tmpFile_dynamics) == -1) {
		printf("ERROR: Couldn't create temporary file!\n");
		return 1;
	}

	jp_type SYNC_POS; // the position each WAM should move to before linking
    SYNC_POS[1] = -0.2;
    SYNC_POS[2] = 0.0;
    SYNC_POS[3] = 0.0;

	//Master Master System
	MasterMaster<DOF> mm(pm.getExecutionManager(), argv[1]);
	systems::connect(wam.jpOutput, mm.input);

	//ID FeedFWD
	double h_omega_p = 15.0;
	systems::FirstOrderFilter<jp_type> hp1;
	hp1.setHighPass(jp_type(h_omega_p), jp_type(h_omega_p));
	systems::FirstOrderFilter<jp_type> hp2;
	hp2.setHighPass(jp_type(h_omega_p), jp_type(h_omega_p));
	systems::Gain<jp_type, double, jv_type> jvDes(1.0);
	systems::Gain<jp_type, double, ja_type> jaDes(1.0);
	jv_type jvLimits;
	jvLimits << 0.1, 1.5, 1.5, 1.5, 0.1, 0.1, 0.1;
	systems::Callback<jv_type> jvSat(boost::bind(saturateJv<DOF>,_1, jvLimits));
	jv_type jaLimits;
	jaLimits << 0.1, 1.5, 1.5, 1.5, 0.1, 0.1, 0.1;
	systems::Callback<ja_type> jaSat(boost::bind(saturateJa<DOF>,_1, jaLimits));
	systems::Callback<ja_type> jaSatCurr(boost::bind(saturateJa<DOF>,_1, jaLimits));
	jt_type jtLimits;
	jtLimits << 10, 10, 10, 10, 0, 0, 0;
	systems::Callback<jt_type> feedSat(boost::bind(saturateJt<DOF>,_1, jtLimits));
	systems::Ramp time(pm.getExecutionManager(), 1.0);
	double coeff = 1.0;
    Dynamics<DOF> feedFWD(coeff);

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

	//ID for arm
	Dynamics<DOF> inverseDyn;
	double h_omega = 15.0;
	systems::FirstOrderFilter<jv_type> hp;
	hp.setHighPass(jv_type(h_omega), jv_type(h_omega));
	systems::Gain<jv_type, double, ja_type> jaCur(1.0);

	connect(wam.jvOutput, hp.input);
	connect(hp.output, jaCur.input);
	connect(jaCur.output, jaSatCurr.input);
	pm.getExecutionManager()->startManaging(hp);
	sleep(1);
	connect(wam.jpOutput, inverseDyn.jpInputDynamics); 
	connect(wam.jvOutput, inverseDyn.jvInputDynamics);
    connect(jaSatCurr.output, inverseDyn.jaInputDynamics);

	//Applied External Torque
	jt_type A;
	A << 0.0, 0.0, 0.0, 0.0;
	double f = 0.5;
	sinextorq<DOF> extorqFeedFWD(A, f);
	systems::Summer<jt_type, 2> feedFwdSum;
	systems::Ramp time(pm.getExecutionManager(), 1.0);
	connect(time.output, extorqFeedFWD.timef);
	connect(feedFWD.dynamicsFeedFWD, feedFwdSum.getInput(0));
	connect(extorqFeedFWD.extorq, feedFwdSum.getInput(1));
	connect(feedFwdSum.output, feedSat.input);

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
	systems::connect(extorqFeedFWD.extorq, tg_dynamics.template getInput<5>());

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
					timelog.start();
					connect(tg_kinematics.output, logger_kinematics.input);
					connect(tg_dynamics.output, logger_dynamics.input);
					printf("Logging started.\n");
				} else {
					printf("WARNING: Linking was unsuccessful.\n");
				}

				//Applying External Torque
				time.stop();
				time.reset();
				time.start();
				btsleep((2/f));
				time.stop();
				time.reset();
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

		case 's':{
				logger_kinematics.closeLog();
				logger_dynamics.closeLog();
				printf("Logging stopped.\n");
				timelog.stop();
				timelog.reset();
				exit_called = true; 
				break;
		}
		default:
			printf("\n");
			printf("    'l' to toggle linking with other WAM\n");
			printf("    't' to tune control gains\n");
			printf("    's' to save data\n");
			break;

	}

	}

	// Create the directory using mkdir() 
	std::string folderName = argv[2];
    if (mkdir(folderName.c_str(), 0777) == -1) {
        std::cerr << "Error: Could not create directory. It might already exist." << std::endl;
        return 1; // Exit the program if directory creation fails
    }

	std::string kinematicsFilename = ".data/" + folderName + "/kinematics.txt";
	std::string dynamicsFilename = ".data/" + folderName + "/dynamics.txt";
	std::string configFilename = ".data/" + folderName + "/config.txt";
	std::ofstream kinematicsFile(kinematicsFilename);
	std::ofstream dynamicsFile(dynamicsFilename);
	std::ofstream configFile(configFilename);

	//Config File Writing
	configFile << "Master Master Teleop with Dynamic Compensation and Sinusoidal External Torque-Follower.\n";
	configFile << "Kinematics data: time, desired joint pos, feedback joint pos, desired joint vel, feedback joint vel, desired joint acc, feedback joint acc.\n";
	configFile << "Dynamics data: time, wam joint torque input, wam gravity input, dynamic feed forward, inverse dynamic, applied external torque.\n";
	configFile << "Joint Position PID Controller: \nkp: " << wam.jpController.getKp() << "\nki: " << wam.jpController.getKi()<<  "\nkd: "<< wam.jpController.getKd() <<"\nControl Signal Limit: " << wam.jpController.getControlSignalLimit() <<".\n";
	configFile << "Sync Pos:" << SYNC_POS;
	configFile << "\nDesired Joint Vel Saturation Limit: " << jvLimits;
	configFile << "\nDesired Joint Acc Saturation Limit: " << jaLimits;
	configFile << "\nCurrent Joint Acc Saturation Limit: " << jaLimits;
	configFile << "\nFeedFwd Torque Saturation Limit: " << jtLimits;
	configFile << "\nHigh Pass Filter Frq used to get desired vel and acc:" << h_omega_p;
	configFile << "\nHigh Pass Filter Frq used to get current acc:" << h_omega;
	configFile << "\nTanh Coeef in Dynamics:" << coeff;
	configFile << "\nFrequency and amplitude for the applied external torque: F:" << f << "A: " << A;

	log::Reader<tuple_type_kinematics> lr_kinematics(tmpFile_kinematics);
	lr_kinematics.exportCSV(kinematicsFile);
	log::Reader<tuple_type_dynamics> lr_Dynamics(tmpFile_dynamics);
	lr_Dynamics.exportCSV(dynamicsFile);
	configFile.close();
	printf("Output written to %s folder.\n", argv[2]);

	std::remove(tmpFile_kinematics);
	std::remove(tmpFile_dynamics);

	return 0;
}


