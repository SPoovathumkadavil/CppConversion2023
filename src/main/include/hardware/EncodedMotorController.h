
#pragma once

#include <frc/geometry/Rotation2d.h>

class EncodedMotorController {
public:
    virtual void SetAngularVelocity(double targetAngularVelocity);
    virtual double GetAngularVelocity();
    virtual void SetAngle(frc::Rotation2d targetAngle);
    virtual double GetAngle();
    virtual void SetOutput(double output);
    virtual double GetOutput();
    virtual EncodedMotorController SetCurrentLimit(int currentLinit);
    virtual EncodedMotorController SetPid(double p, double i, double d);
	virtual EncodedMotorController setMinAngle(double minAngle);
	virtual EncodedMotorController setMaxAngle(double maxAngle);
	virtual EncodedMotorController setMinOutput(double minOutput);
	virtual EncodedMotorController setMaxOutput(double maxOutput);
	virtual EncodedMotorController setInversion(bool shouldInvert);
	virtual EncodedMotorController setBrakeOnIdle(bool shouldBrake);
	virtual EncodedMotorController setAngleTolerance(double tolerance);
};
