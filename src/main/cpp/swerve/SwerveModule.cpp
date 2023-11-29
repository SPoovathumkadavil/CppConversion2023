
#include "swerve/SwerveModule.h"
#include <frc/kinematics/SwerveModuleState.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>
#include <frc/motorcontrol/Talon.h>
#include "constants.h"

using namespace units::angle;
using namespace units::velocity;
using namespace constants::swerve;

frc::SwerveModuleState SwerveModule::getModuleState()
{
    return {
        meters_per_second_t{driveMotor.GetSelectedSensorVelocity() / 2048 * 10 * DRIVE_RATIO * WHEEL_DIAMETER_METERS / 2},
        frc::Rotation2d(radian_t{angleMotor.GetSelectedSensorPosition() / 2048 * ANGLE_RATIO})
    };
}

/// @brief Adjusts the target angle and speed based on the current angle of the swerve module.
///        If the difference between the target angle and current angle is greater than 90 degrees,
//         the target speed is negated and the target angle is adjusted by 180 degrees.
/// @param targetAngle
/// @param targetVelocity
/// @param currentAngle
/// @return
frc::SwerveModuleState SwerveModule::adjustTargetAngleAndSpeed(frc::Rotation2d targetAngle, meters_per_second_t targetVelocity, frc::Rotation2d currentAngle)
{
    units::angle::degree_t delta = targetAngle.Degrees() - currentAngle.Degrees();
    if (abs(delta.value()) > 90)
    {
        targetVelocity = -targetVelocity;
        targetAngle = delta > 90_deg ? (targetAngle.RotateBy(frc::Rotation2d(-180_deg))) : (targetAngle.RotateBy(frc::Rotation2d(180_deg)));
    }
    return {targetVelocity, targetAngle};
}

/// @brief Places the given angle in the appropriate 0 to 360 degree scope based on the reference angle.
/// @param scopeReference
/// @param newAngle
/// @return
degree_t SwerveModule::placeInAppropriate0To360Scope(degree_t scopeReference, degree_t newAngle)
{
    auto delta = newAngle - scopeReference;
    delta += 180_deg;                           // shift range to [0, 360]
    delta = degree_t{fmod(delta.value(), 360)}; // normalize to [0, 360]
    if (delta.value() < 0)
        delta += 360_deg; // correct negative values
    delta -= 180_deg;     // shift range back to [-180, 180]
    return scopeReference + delta;
}

/// @brief Minimize the change in heading the desired swerve module state would require by potentially
///        reversing the direction the wheel spins. Customized from WPILib's version to include placing
//         in appropriate scope for CTRE onboard control.
/// @param desiredState
/// @param currentAngle
/// @return
frc::SwerveModuleState SwerveModule::optimizeTalon(frc::SwerveModuleState desiredState, frc::Rotation2d currentAngle)
{
    degree_t targetAngle = placeInAppropriate0To360Scope(degree_t{currentAngle.Degrees()}, degree_t{desiredState.angle.Degrees()});
    meters_per_second_t targetSpeed = desiredState.speed;
    return adjustTargetAngleAndSpeed(targetAngle, targetSpeed, currentAngle.Degrees());
}
