#ifndef DYNAMICS_H_
#define DYNAMICS_H_

#include <barrett/math/traits.h>
#include <list>
#include <barrett/units.h>
#include <barrett/detail/ca_macro.h>
#include <barrett/systems/abstract/system.h>

#include <eigen3/Eigen/Core>
#include <libconfig.h++>

#include <barrett/detail/ca_macro.h>
#include <barrett/math/traits.h>
#include <barrett/systems/abstract/execution_manager.h>
#include <barrett/systems/abstract/controller.h>

// #include <PI_4D.hpp>
// #include <Y_4D.hpp>

#include <PI_4D_nogravity.hpp>
#include <Y_4D_nogravity.hpp>

using namespace barrett;

template<size_t DOF>
class Dynamics: public systems::System {

	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	/* Torque*/
public:
	Input<jp_type> jpInputDynamics;
	Input<jv_type> jvInputDynamics;
    Input<ja_type> jaInputDynamics;

public:
	Output<jt_type> dynamicsFeedFWD;
protected:
	typename Output<jt_type>::Value* dynamicsFeedFWDValue;

public:
	explicit Dynamics(/*systems::ExecutionManager* em*/) :
			jpInputDynamics(this), jvInputDynamics(this), jaInputDynamics(this), dynamicsFeedFWD(this,
					&dynamicsFeedFWDValue){
	//		      em->startManaging(*this);
//		    }
	}

	virtual ~Dynamics() {
		this->mandatoryCleanUp();
	}

protected:
	jp_type tmp_theta_pos;
	jv_type tmp_theta_vel;
	ja_type tmp_theta_acc;
	Eigen::Matrix<double,2 ,8> Y;
	Eigen::Matrix<double, 8, 1> P;
	Eigen::Vector4d ThetaInput;
	Eigen::Vector4d ThetadotInput;
	Eigen::Vector4d ThetaddotInput;
	Eigen::Vector2d FeedFwd;
	jt_type dynFeedFWD;
	double coeff;

	virtual void operate() {
		tmp_theta_pos = this->jpInputDynamics.getValue();
		ThetaInput << tmp_theta_pos[0], tmp_theta_pos[1], tmp_theta_pos[2], tmp_theta_pos[3];
		tmp_theta_vel = this->jvInputDynamics.getValue();
		ThetadotInput << tmp_theta_vel[0], tmp_theta_vel[1], tmp_theta_vel[2], tmp_theta_vel[3];
		tmp_theta_acc = this->jaInputDynamics.getValue();
		ThetaddotInput << tmp_theta_acc[0], tmp_theta_acc[1], tmp_theta_acc[2], tmp_theta_acc[3];
		Y = calculate_Y_matrix(ThetaInput, ThetadotInput, ThetaddotInput);
		P = initialize_pi();
		
		FeedFwd = Y * P;
	
		dynFeedFWD[0] = FeedFwd[0];
		dynFeedFWD[2] = FeedFwd[1];

		this->dynamicsFeedFWDValue->setData(&dynFeedFWD);

	}
private:
	DISALLOW_COPY_AND_ASSIGN(Dynamics);
};
#endif /* DYNAMICS_H_ */