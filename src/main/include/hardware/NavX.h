
#pragma once

#include <AHRS.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/MathUtil.h>

class NavX {
public:
    NavX(frc::I2C::Port kmxp) : ahrs(kmxp), gyroZero(frc::Rotation2d()) {};
    NavX(frc::SPI::Port kmxp) : ahrs(kmxp), gyroZero(frc::Rotation2d()) {};

    AHRS *GetAHRS()
    {
        return &ahrs;
    }

    frc::Rotation2d GetUnwrappedAngle()
    {
        return ahrs.GetRotation2d();
    }

    /// @brief Interval: [-pi, pi]
    /// @return 
    frc::Rotation2d GetAngle()
    {
        return GetYaw();
    }

    /// @brief Interval: [-pi, pi]
    /// @return 
    frc::Rotation2d GetOffsetedAngle()
    {
        return WrapRotation2d(GetAngle() - GetGyroZero());
    }

    /// @brief Interval: [-pi, pi]
    /// @return 
    frc::Rotation2d GetYaw()
    {
        return frc::Rotation2d(units::degree_t{-ahrs.GetYaw()});
    }

    /// @brief Interval: [-pi, pi]
    /// @return 
    frc::Rotation2d GetPitch()
    {
        return frc::Rotation2d(units::degree_t{-ahrs.GetPitch()});
    }

    /// @brief Interval: [-pi, pi]
    /// @return 
    frc::Rotation2d GetRoll()
    {
        return frc::Rotation2d(units::degree_t{-ahrs.GetRoll()});
    }

    /// @brief Interval: [-pi, pi]
    frc::Rotation2d GetGyroZero()
    {
        return gyroZero;
    }

    void ZeroGyro()
    {
        gyroZero = frc::Rotation2d();
    }

    void ZeroGyroWithOffset(frc::Rotation2d offset)
    {
        gyroZero = WrapRotation2d(GetAngle() - offset);
    }

    void SetGyroZero(frc::Rotation2d newZero)
    {
        gyroZero = WrapRotation2d(newZero);
    }

    static frc::Rotation2d WrapRotation2d(frc::Rotation2d rotationToWrap)
    {
        return frc::Rotation2d(frc::AngleModulus(rotationToWrap.Radians()));
    }

private:
    AHRS ahrs;
    frc::Rotation2d gyroZero;
};