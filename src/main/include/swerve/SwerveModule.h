#pragma once

#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include "ctre/Phoenix.h"
#include <frc/motorcontrol/Talon.h>
#include <frc/geometry/Translation2d.h>

class SwerveModuleConfig
{
public:
    SwerveModuleConfig(
        int driveId, 
        int angleId, 
        double dkp, 
        double dki, 
        double dkd, 
        double akp, 
        double aki, 
        double akd, 
        bool angleInvert, 
        frc::Translation2d translationToCenter
    ) : driveId(driveId), angleId(angleId), dkp(dkp), dki(dki), dkd(dkd), akp(akp), aki(aki), akd(akd), angleInvert(angleInvert), translationToCenter(translationToCenter) {};
    int driveId;
    int angleId;
    double dkp;
    double dki;
    double dkd;
    double akp;
    double aki;
    double akd;
    bool angleInvert = false;
    frc::Translation2d translationToCenter;
};

class SwerveModule
{
public:
    SwerveModule(SwerveModuleConfig config) : driveMotor(config.driveId), angleMotor(config.angleId), translationToCenter(config.translationToCenter)
    {
        driveMotor.Config_kP(0, config.dkp);
        driveMotor.Config_kI(0, config.dki);
        driveMotor.Config_kD(0, config.dkd);
        angleMotor.Config_kP(0, config.akp);
        angleMotor.Config_kI(0, config.aki);
        angleMotor.Config_kD(0, config.akd);
        angleMotor.SetInverted(config.angleInvert);
    }
    void Drive(frc::SwerveModuleState state);
    frc::SwerveModuleState GetModuleState();
    frc::SwerveModulePosition GetModulePosition();
    void SetModuleAngle(frc::Rotation2d targetAngle);
    void SetModuleVelocity(units::velocity::meters_per_second_t targetVelocity);

    frc::Translation2d GetTranslationFromCenter() { return translationToCenter; };

    double GetAngularVelocity();

private:
    frc::SwerveModuleState AdjustTargetAngleAndSpeed(frc::Rotation2d targetAngle, units::velocity::meters_per_second_t targetVelocity, frc::Rotation2d currentAngle);
    units::angle::degree_t PlaceInAppropriate0To360Scope(units::angle::degree_t scopeReference, units::angle::degree_t newAngle);
    frc::SwerveModuleState OptimizeTalon(frc::SwerveModuleState desiredState, frc::Rotation2d currentAngle);

    frc::Translation2d translationToCenter;

    WPI_TalonFX driveMotor;
    WPI_TalonFX angleMotor;
};
