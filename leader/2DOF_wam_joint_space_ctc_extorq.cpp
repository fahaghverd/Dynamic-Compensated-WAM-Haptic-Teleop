/*
 * 2DOF_wam_joint_space_ctc.cpp 
 *
 *  Created on: Aug, 2024
 *      Author: Faezeh
 */


#include <Dynamics.hpp>
#include <sin_jp_trajectory.hpp>
#include <unistd.h>
#include <iostream>
#include <string>
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>
#include <barrett/detail/stl_utils.h>
#include <barrett/log.h>

#define BARRETT_SMF_WAM_CONFIG_PATH "wam3noJ1"
#include <barrett/standard_main_function.h>

using namespace barrett;

//Our joint space Computed Torque Controller(CTC) : id_feedFWD + gravity + PD controller
template<size_t DOF>
class JsCTCController :  public systems::System{ //
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

// IO  (inputs)
public:
	Input<jp_type> refJPInput;	// reference joint pos. input   
	Input<jv_type> refJVInput;	// reference joint vel. input

public:
	Input<jp_type> feedbackjpInput;     // joint pos. input       
	Input<jv_type> feedbackjvInput;  	// joint vel. input

public:
	Input<jt_type> feedFWDInput; //Inverse Dynamic Feed FWD
	Input<jt_type> gravityInput; //Gravity Term Input from WAM

// IO  (outputs)
public:
	Output<jt_type> controlJtOutput;    // Output Control Joint Torque

protected:
	typename Output<jt_type>::Value* controlJtOutputValue;

public:
jt_type computedT;

public:
	explicit JsCTCController(Eigen::MatrixXd proportionalGains, Eigen::MatrixXd dampingGains/*, float kpCoeff, float KdCoeff*/, const std::string& sysName = "JsCTCController"):
		System(sysName), refJPInput(this), refJVInput(this), feedbackjpInput(this), feedbackjvInput(this), controlJtOutput(this, &controlJtOutputValue), feedFWDInput(this), gravityInput(this), kp(proportionalGains), kd(dampingGains)/*, kpCf(kpCoeff), kdCf(KdCoeff)*/{}

	virtual ~JsCTCController() { this->mandatoryCleanUp(); }

protected:
	float kpCf, kdCf; 
	sqm_type kp, kd;
	jt_type jt_out, feedFWD, gravity;
	jp_type jp_sys, jp_ref;
	jv_type jv_sys, jv_ref;
	ja_type ja_ref;

	virtual void operate() {
		/*Taking reference values from the input terminal of this system*/
		jp_ref = this->refJPInput.getValue();
		jv_ref = this->refJVInput.getValue();

		/*Taking feedback values from the input terminal of this system*/
		jp_sys = this->feedbackjpInput.getValue();
		jv_sys = this->feedbackjvInput.getValue();

		/*Taking feed forward term from the input terminal of this system*/
		feedFWD = this->feedFWDInput.getValue();
		gravity = this->gravityInput.getValue();

		gravity[1] = 0.0;
		gravity[3] = 0.0;
		// feedFWD[3] = 0.0;


		jt_out = 1.0 * feedFWD + gravity + 0.0 * kp * (jp_ref - jp_sys) + 0.0 * kd * (jv_ref - jv_sys);

		computedT = jt_out;
		
		this->controlJtOutputValue->setData(&jt_out);
	}

private:
	DISALLOW_COPY_AND_ASSIGN(JsCTCController);
};

//Creating a templated multiplier for our real-time computation
template<typename T1, typename T2, typename OutputType>
  class Multiplier : public systems::System, public systems::SingleOutput<OutputType>
  {
  public:
    Input<T1> input1;
  public:
    Input<T2> input2;

  OutputType Data;
  public:
    Multiplier(std::string sysName = "Multiplier") :
        systems::System(sysName), systems::SingleOutput<OutputType>(this), input1(this), input2(this)
    {
    }
    virtual ~Multiplier()
    {
      mandatoryCleanUp();
    }

  protected:
    OutputType data;
    virtual void operate()
    {
      data = input1.getValue() * input2.getValue();
      Data = data;
      this->outputValue->setData(&data);
    }

  private:
    DISALLOW_COPY_AND_ASSIGN(Multiplier);

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };


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


// Function to split a string by a delimiter and return a vector of the elements
std::vector<std::string> split(const std::string& str, char delimiter) {
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(str);
    while (std::getline(tokenStream, token, delimiter)) {
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
        Eigen::VectorXd eigenVec(stringElements.size());
        for (size_t i = 0; i < stringElements.size(); ++i) {
            std::istringstream iss(stringElements[i]);
            double value;
            if (iss >> value) {
                eigenVec(i) = value;
            } else {
                std::cerr << "Invalid value in vector for " << varName << std::endl;
                return defaultValue;
            }
        }
        return eigenVec;
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

    minRatio = (limit.array() / (x.cwiseAbs()).array()).minCoeff(&index);
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

    minRatio = (limit.array() / (x.cwiseAbs()).array()).minCoeff(&index);
    if (minRatio < 1.0) {
        return minRatio * x;
    } else {
        return x;
    }
}

template <size_t DOF>
typename units::JointTorques<DOF>::type saturateJt(const typename units::JointTorques<DOF>::type& x, const typename units::JointTorques<DOF>::type& limit) {
	int index;
	double minRatio;

	minRatio = (limit.array() / (x.cwiseAbs()).array()).minCoeff(&index);
	if (minRatio < 1.0) {
		return minRatio * x;
	} else {
		return x;
	}
}


template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm,	systems::Wam<DOF>& wam) {
    BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	char tmpFile[] = "/tmp/btXXXXXX";
	if (mkstemp(tmpFile) == -1) {
		printf("ERROR: Couldn't create temporary file!\n");
		return 1;
	}

	pm.getSafetyModule()->setVelocityLimit(1.5);
	wam.gravityCompensate(true);
	// btsleep(25000);
	//Moving to start pose
	jp_type start_pos;
	start_pos<< 0.0, 0.0, 0.3;
	wam.moveTo(start_pos);

	//Trapezoidal vel traj
	// jp_type end_pos;
	// end_pos<< 0.0, 0.0, 0.0, -0.5, 0.0, 0.0, 0.0;
	// std::vector<jp_type, Eigen::aligned_allocator<jp_type> > vec;
	// vec.push_back(start_pos);
	// vec.push_back(end_pos);
	// math::Spline<jp_type> spline(vec);
	// math::TrapezoidalVelocityProfile profile(0.0, 0.0, 0.0, spline.changeInS());
	// systems::Callback<double, jp_type> refTraj(boost::bind(boost::ref(spline), boost::bind(boost::ref(profile), _1)));

	// Sinosoidal Traj
	std::vector<float> inputs;
	float temp;
	std::cout<<"Enter A2, A4, and f"<<std::endl;
	
	for (int i = 0; i < 3; ++i) {
        std::cin >> temp;
        inputs.push_back(temp);  // Add the number to the vector
    }

	v_type a;
	a << 0.4, 0.0 , 0.0;
	double F = 0.2;
	sinJpRefTrajectory<DOF> refTraj(start_pos, a, F);

	// //Const Vel Trajectory
	// jv_type thetad_des;
    // thetad_des << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
	// std::vector<float> inputs;
	// float temp;
	// std::cout<<"Enter setpoit 2 and 4"<<std::endl;
	// for (int i = 0; i < 2; ++i) {
    //     std::cin >> temp;
    //     inputs.push_back(temp);  // Add the number to the vector
    // }
	// start_pos << 0.0, inputs[0], 0.0, inputs[1], 0.0, 0.0, 0.0;
    // constVelRefTrajectory<DOF> refTraj(start_pos, thetad_des);

	// // float kpCf, kdCf;
	// v_type kp,kd; 
	// std::vector<float> inputs1;
	// std::cout<<"Enter kp 2 and 4, kd 2 and 4"<<std::endl;
	// for (int i = 0; i < 4; ++i) {
    //     std::cin >> temp;
    //     inputs1.push_back(temp);  // Add the number to the vector
    // }
	// kp << 900, inputs1[0],  600,  inputs1[1],   50,   50,    8;
	// kd << 10,   inputs1[2],    5,    inputs1[3],  0.5,  0.5, 0.05;
	// 	kp << 900, 2500,  600,  500,   50,   50,    8; //250 50
	// kd << 10,   20,    5,    2,  0.5,  0.5, 0.05; //10 1
	// JsCTCController<DOF> compTorqueController(kp.asDiagonal(),kd.asDiagonal());//, inputs1[0], inputs1[1]);
	// Dynamics<DOF> wam2dofDynamics;
	jt_type jtLimits;
	jtLimits << 25, 20, 15;
	systems::Callback<jt_type> jtSat(boost::bind(saturateJt<DOF>,_1, 2*jtLimits));
	systems::Ramp time(pm.getExecutionManager(), 1.0);
    
	// double omega_p = 1000.0;
	// systems::FirstOrderFilter<jp_type> hp1;
	// hp1.setHighPass(jp_type(omega_p), jp_type(omega_p));
	// systems::FirstOrderFilter<jp_type> hp2;
	// hp2.setHighPass(jp_type(omega_p), jp_type(omega_p));
	// systems::Gain<jp_type, double, jv_type> jvDes(1.0);
	// systems::Gain<jp_type, double, ja_type> jaDes(1.0);

	systems::connect(time.output, refTraj.timef);
	//systems::connect(time.output, refTraj.input);
	// connect(trefTraj.output, hp1.input);
	// connect(hp1.output, hp2.input);
	// connect(hp2.output, jaDes.input);
	// connect(hp1.output, jvDes.input);
	// pm.getExecutionManager()->startManaging(hp2);
	// sleep(1);

	// systems::connect(refTraj.output, compTorqueController.refJPInput);
    // systems::connect(jvDes.output, compTorqueController.refJVInput);	

    // systems::connect(refTraj.output, wam2dofDynamics.jpInputDynamics);
	// systems::connect(jvDes.output, wam2dofDynamics.jvInputDynamics);
    // systems::connect(jaDes.output, wam2dofDynamics.jaInputDynamics);


    // systems::connect(refTraj.referencePTrack, compTorqueController.refJPInput);
    // systems::connect(refTraj.referenceVTrack, compTorqueController.refJVInput);	


	//ID for arm dynamics
	double coeff = 10;
	double h_omega_p = 15;
	Dynamics<DOF> inverseDyn;
	systems::FirstOrderFilter<jp_type> hp3;
	systems::FirstOrderFilter<jp_type> hp4;
	hp3.setHighPass(jp_type(h_omega_p), jp_type(h_omega_p));
	hp4.setHighPass(jp_type(h_omega_p), jp_type(h_omega_p));
	systems::Gain<jp_type, double, ja_type> jaCur(1.0);
	systems::Gain<jp_type, double, jv_type> jvCur(1.0);
	jv_type jvLimits_default;
	jvLimits_default << 2, 2, 2;
	jv_type jvLimits = jv_type(getEnvEigenVector<DOF>("jv_limits", v_type(jvLimits_default)));
	systems::Callback<jv_type> jvSat(boost::bind(saturateJv<DOF>,_1, jvLimits));
	ja_type jaLimits_default;
	jaLimits_default << 2, 2, 2; //Increaseade just to discard it
	ja_type jaLimits = ja_type(getEnvEigenVector<DOF>("ja_limits", v_type(jaLimits_default)));
	systems::Callback<ja_type> jaSat(boost::bind(saturateJa<DOF>,_1, jaLimits));

	connect(wam.jpOutput, hp3.input);
	connect(hp3.output, hp4.input);
	connect(hp4.output, jaCur.input);
	connect(jvCur.output, jvSat.input);
	connect(hp3.output, jvCur.input);
	connect(jaCur.output, jaSat.input);
	pm.getExecutionManager()->startManaging(hp4);
	sleep(1);
	connect(wam.jpOutput, inverseDyn.jpInputDynamics); 
	connect(jvCur.output, inverseDyn.jvInputDynamics);
    connect(jaCur.output, inverseDyn.jaInputDynamics);

	//Applied External Torque
	jt_type A_default;
	A_default << 4.0, 0.0, 0.0;
	jt_type A = jt_type(getEnvEigenVector<DOF>("A", v_type(A_default)));
	double f_default = 0.5;
	double f = getEnvDouble("f", f_default);
	sinextorq<DOF> extorqFeedFWD(A, f);
	systems::Summer<jt_type, 2> feedFwdSum;

	systems::connect(time.output, extorqFeedFWD.timef);
	// systems::connect(inverseDyn.dynamicsFeedFWD, feedFwdSum.getInput(0));
	// systems::connect(extorqFeedFWD.extorq, feedFwdSum.getInput(1));

   	// systems::connect(wam.gravity.output, compTorqueController.gravityInput);
    // systems::connect(feedFwdSum.output, compTorqueController.feedFWDInput);
	// systems::connect(wam.jpOutput, compTorqueController.feedbackjpInput);
	// systems::connect(wam.jvOutput, compTorqueController.feedbackjvInput);
    // systems::connect(compTorqueController.controlJtOutput, jtSat.input);
	// connect(feedFwdSum.output, jtSat.input);
	connect(extorqFeedFWD.extorq, jtSat.input);


//	RT Logging stuff
	systems::Ramp timelog(pm.getExecutionManager(), 1.0);
	systems::TupleGrouper<double, jp_type, jt_type, jt_type, jt_type, jt_type, jv_type, ja_type> tg;
	systems::connect(timelog.output, tg.template getInput<0>());
	systems::connect(wam.jpOutput, tg.template getInput<1>());
	systems::connect(wam.jtSum.output, tg.template getInput<2>());
	systems::connect(wam.gravity.output, tg.template getInput<3>());
	systems::connect(extorqFeedFWD.extorq, tg.template getInput<4>());
	systems::connect(inverseDyn.dynamicsFeedFWD, tg.template getInput<5>());
	systems::connect(jvCur.output, tg.template getInput<6>());
	systems::connect(jaCur.output, tg.template getInput<7>());
	// connect(refTraj.refPTrack, tg.template getInput<8>());



	typedef boost::tuple<double, jp_type, jt_type, jt_type, jt_type, jt_type, jv_type, ja_type> tuple_type;
	const size_t PERIOD_MULTIPLIER = 1;
	systems::PeriodicDataLogger<tuple_type> logger(
			pm.getExecutionManager(),
			new log::RealTimeWriter<tuple_type>(tmpFile, PERIOD_MULTIPLIER * pm.getExecutionManager()->getPeriod()),
			PERIOD_MULTIPLIER);
	
	time.stop();
	time.reset();


	std::cout<<"Press [Enter] to start."<<std::endl;
	detail::waitForEnter();

	timelog.stop();
	timelog.reset();
	timelog.start();
	systems::connect(tg.output, logger.input);
	printf("Logging started.\n");
	
	wam.idle();
	time.start();
	wam.trackReferenceSignal(jtSat.output); // Put it after time start so that feefwd term is active when we start applying torque.
	// connect(jtSat.output, wam.input);

	std::cout<<"Press [Enter] to stop."<<std::endl;
	detail::waitForEnter();
	time.stop();

	timelog.stop();
	logger.closeLog();
	printf("Logging stopped.\n");

	log::Reader<tuple_type> lr(tmpFile);
	lr.exportCSV(argv[1]);
	printf("Output written to %s.\n", argv[1]);
	std::remove(tmpFile);

	wam.moveHome();

	// Wait for the user to press Shift-idle
	pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
	return 0;
}