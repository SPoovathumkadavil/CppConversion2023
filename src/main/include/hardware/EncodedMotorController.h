
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
    virtual EncodedMotorController *SetCurrentLimit(int currentLinit);
    virtual EncodedMotorController *SetPID(double p, double i, double d);
	virtual EncodedMotorController *SetMinAngle(double minAngle);
	virtual EncodedMotorController *SetMaxAngle(double maxAngle);
	virtual EncodedMotorController *SetMinOutput(double minOutput);
	virtual EncodedMotorController *SetMaxOutput(double maxOutput);
	virtual EncodedMotorController *SetInversion(bool shouldInvert);
	virtual EncodedMotorController *SetBrakeOnIdle(bool shouldBrake);
	virtual EncodedMotorController *SetAngleTolerance(double tolerance);
};
