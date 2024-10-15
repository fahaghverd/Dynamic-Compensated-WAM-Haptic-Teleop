#pragma once

#include <barrett/units.h>
#include <barrett/systems.h>
#include <cmath>

template<size_t DOF>
class sinJpRefTrajectory: public systems::System {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

// IO
public:
	Input<double> timef;
	Output<jp_type> referencePTrack;
	Output<jv_type> referenceVTrack;
	Output<ja_type> referenceATrack;

protected:
	typename System::Output<jp_type>::Value* referencePOpValue;
	typename System::Output<jv_type>::Value* referenceVOpValue;
	typename System::Output<ja_type>::Value* referenceAOpValue;

public:
	explicit sinJpRefTrajectory(jp_type start_pose, v_type A, double f, const std::string& sysName = "sinJpRefTrajectory"):
		System(sysName), timef(this), referencePTrack(this, &referencePOpValue), referenceVTrack(this, &referenceVOpValue),
		referenceATrack(this, &referenceAOpValue), start_pose(start_pose), A(A), f(f){}

	virtual ~sinJpRefTrajectory() { this->mandatoryCleanUp(); }

protected:
    double f, t;
    v_type A;
	jp_type start_pose, refPTrack;
	jv_type refVTrack;
	ja_type refATrack;

	virtual void operate() {
        t = this->timef.getValue();
        refPTrack = A * cos(2 * M_PI * f* t) - A + start_pose;
        refVTrack = -1 * A * 2 * M_PI * f * sin(2 * M_PI * f * t);
        refATrack = -1 * A * pow(2 * M_PI * f, 2) * cos(2 * M_PI * f * t);

		referencePOpValue->setData(&refPTrack);
		referenceVOpValue->setData(&refVTrack);
		referenceAOpValue->setData(&refATrack);
	}

private:
	DISALLOW_COPY_AND_ASSIGN(sinJpRefTrajectory);
};
