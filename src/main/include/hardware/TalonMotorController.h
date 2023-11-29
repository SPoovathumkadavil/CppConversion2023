
#pragma once

#include "hardware/EncodedMotorController.h"
#include <ctre/Phoenix.h>

class TalonMotorController : public EncodedMotorController
{

public:

    enum TalonModel {
        TalonFX,
        TalonSRX
    };

    double encoderTicks[2];

    TalonMotorController(int deviceId, TalonModel model) : innerTalon(deviceId, model == TalonFX ? "Talon FX" : "Talon SRX"), model(model) 
    {
        encoderTicks[TalonFX] = 2048 / M_PI / 2;
        encoderTicks[TalonSRX] = 4096 / M_PI / 2;
        switch (model) {
            case (TalonFX):
                innerTalon.Config_IntegralZone(0, 0);
                innerTalon.ConfigMotionCruiseVelocity(10000);
                innerTalon.ConfigMotionAcceleration(10000);
                innerTalon.ConfigAllowableClosedloopError(0, 0);
                innerTalon.ConfigClearPositionOnQuadIdx(true, 10);
                break;
            default:
                break;
        }
    };

    void SetOutput(double targetPercentOutput) override
    {
        innerTalon.Set(
            ctre::phoenix::motorcontrol::ControlMode::PercentOutput,
            targetPercentOutput
        );
    }

    double GetOutput() override
    {
        return innerTalon.GetMotorOutputPercent();
    }

    void SetAngularVelocity(double targetAngularVelocity) override
    {
        innerTalon.Set(
            ctre::phoenix::motorcontrol::ControlMode::Velocity,
            targetAngularVelocity * encoderTicks[model] / 10
        );
    }

    double GetAngularVelocity() override
    {
        return innerTalon.GetSelectedSensorVelocity() / encoderTicks[model] * 10;
    }

    void SetAngle(frc::Rotation2d targetRotation) override
    {
        switch(model) {
            case TalonFX:
                innerTalon.Set(
                    ctre::phoenix::motorcontrol::
                );
        }
    }

private:
    TalonModel model;
    ctre::phoenix::motorcontrol::can::BaseTalon innerTalon;
};