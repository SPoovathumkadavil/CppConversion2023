
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

    double encoderTicksPerRadian[2];

    TalonMotorController(int deviceId, TalonModel model) : innerTalon(deviceId, model == TalonFX ? "Talon FX" : "Talon SRX"), model(model) 
    {
        encoderTicksPerRadian[TalonFX] = 2048 / M_PI / 2;
        encoderTicksPerRadian[TalonSRX] = 4096 / M_PI / 2;
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
            targetAngularVelocity * encoderTicksPerRadian[model] / 10
        );
    }

    double GetAngularVelocity() override
    {
        return innerTalon.GetSelectedSensorVelocity() / encoderTicksPerRadian[model] * 10;
    }

    void SetAngle(frc::Rotation2d targetRotation) override
    {
        switch(model) {
            case TalonFX:
                innerTalon.Set(
                    ctre::phoenix::motorcontrol::ControlMode::MotionMagic,
                    (targetRotation.Radians() * encoderTicksPerRadian[model]).value()
                );
                break;
            case TalonSRX:
                innerTalon.Set(
                    ctre::phoenix::motorcontrol::ControlMode::Position,
                    (targetRotation.Radians() * encoderTicksPerRadian[model]).value()
                );
                break;
        }
    }

    double GetAngle() override
    {
        return innerTalon.GetSelectedSensorPosition() / encoderTicksPerRadian[model];
    }

    EncodedMotorController *SetCurrentLimit(int currentLimit) override
    {
        innerTalon.ConfigSupplyCurrentLimit(
            SupplyCurrentLimitConfiguration(
                true,
                currentLimit,
                currentLimit + 1,
                0.1
            ),
            50
        );
        return this;
    }

    EncodedMotorController *SetPid(double p, double i, double d) override
    {
        innerTalon.Config_kP(0, p);
        innerTalon.Config_kI(0, i);
        innerTalon.Config_kD(0, d);
        return this;
    }

    EncodedMotorController *SetMinAngle(double minPosition) override
    {
        innerTalon.ConfigReverseSoftLimitEnable(true);
        innerTalon.ConfigReverseSoftLimitThreshold(minPosition * encoderTicksPerRadian[model]);
        return this;
    }

    EncodedMotorController *SetMaxAngle(double maxPosition) override
    {
        innerTalon.ConfigForwardSoftLimitEnable(true);
        innerTalon.ConfigForwardSoftLimitThreshold(maxPosition * encoderTicksPerRadian[model]);
        return this;
    }

    EncodedMotorController *SetMinOutput(double minOutput) override
    {
        innerTalon.ConfigPeakOutputReverse(minOutput);
        return this;
    }

    EncodedMotorController *SetMaxOutput(double maxOutput) override
    {
        innerTalon.ConfigPeakOutputForward(maxOutput);
        return this;
    }

    EncodedMotorController *SetInversion(bool shouldInvert) override
    {
        innerTalon.SetInverted(shouldInvert);
        return this;
    }

    EncodedMotorController *SetBrakeOnIdle(bool shouldBrake) override
    {
        innerTalon.SetNeutralMode(
            shouldBrake
            ? NeutralMode::Brake
            : NeutralMode::Coast
        );
        return this;
    }

    EncodedMotorController *SetAngleTolerance(double tolerance) override
    {
        innerTalon.ConfigAllowableClosedloopError(0, tolerance * encoderTicksPerRadian[model]);
        return this;
    }

private:
    TalonModel model;
    ctre::phoenix::motorcontrol::can::BaseTalon innerTalon;
};