
#pragma once

#include "hardware/TalonMotorController.h"
#include <frc/geometry/Translation2d.h>

namespace constants
{
    namespace can
    {
        static int SWERVE_FRONT_LEFT_DRIVE_ID = 3;
        static int SWERVE_FRONT_LEFT_ANGLE_ID = 7;
        static int SWERVE_FRONT_RIGHT_DRIVE_ID = 4;
        static int SWERVE_FRONT_RIGHT_ANGLE_ID = 8;
        static int SWERVE_BACK_LEFT_DRIVE_ID = 2;
        static int SWERVE_BACK_LEFT_ANGLE_ID = 6;
        static int SWERVE_BACK_RIGHT_DRIVE_ID = 9;
        static int SWERVE_BACK_RIGHT_ANGLE_ID = 5;
    }
    namespace swerve
    {
        static double ANGLE_RATIO = 1 / 6.75;
        static double DRIVE_RATIO = 1 / 5.0;
        static double MAX_LINEAR_SPEED_MPS = 5.088;
        static double WHEEL_DIAMETER_METERS = 0.0762;

        static EncodedMotorController *FRONT_LEFT_DRIVE_MOTOR =
            TalonMotorController(
                can::SWERVE_FRONT_LEFT_DRIVE_ID,
                TalonMotorController::TalonFX
            ).SetInversion(true)
                ->SetCurrentLimit(35)
                ->SetPID(0.075, 0, 0);
        static EncodedMotorController *FRONT_LEFT_ANGLE_MOTOR =
            TalonMotorController(
                can::SWERVE_FRONT_LEFT_ANGLE_ID,
                TalonMotorController::TalonFX
            ).SetInversion(false)
                ->SetCurrentLimit(25)
                ->SetPID(0.3, 0, 0);
        static frc::Translation2d FRONT_LEFT_MODULE_TRANSLATION(
            0.3175_m,
            0.2413_m
        );
        
        static EncodedMotorController *FRONT_RIGHT_DRIVE_MOTOR =
            TalonMotorController(
                can::SWERVE_FRONT_RIGHT_DRIVE_ID,
                TalonMotorController::TalonFX
            ).SetInversion(false)
                ->SetCurrentLimit(35)
                ->SetPID(0.05, 0, 0);
        static EncodedMotorController *FRONT_LEFT_ANGLE_MOTOR =
            TalonMotorController(
                can::SWERVE_FRONT_LEFT_ANGLE_ID,
                TalonMotorController::TalonFX
            ).SetInversion(false)
                ->SetCurrentLimit(25)
                ->SetPID(0.3, 0, 0);
        static frc::Translation2d FRONT_RIGHT_MODULE_TRANSLATION(
            0.3175_m,
            -0.2413_m
        );

        static EncodedMotorController *BACK_LEFT_DRIVE_MOTOR =
            TalonMotorController(
                can::SWERVE_BACK_LEFT_DRIVE_ID,
                TalonMotorController::TalonFX
            ).SetInversion(true)
                ->SetCurrentLimit(35)
                ->SetPID(0.075, 0, 0);
        static EncodedMotorController *FRONT_LEFT_ANGLE_MOTOR =
            TalonMotorController(
                can::SWERVE_FRONT_LEFT_ANGLE_ID,
                TalonMotorController::TalonFX
            ).SetInversion(false)
                ->SetCurrentLimit(25)
                ->SetPID(0.25, 0, 0);
        static frc::Translation2d BACK_LEFT_MODULE_TRANSLATION(
            -0.3175_m,
            0.2413_m
        );

        static EncodedMotorController *BACK_RIGHT_DRIVE_MOTOR =
            TalonMotorController(
                can::SWERVE_BACK_RIGHT_DRIVE_ID,
                TalonMotorController::TalonFX
            ).SetInversion(false)
                ->SetCurrentLimit(35)
                ->SetPID(0.05, 0, 0);
        static EncodedMotorController *FRONT_LEFT_ANGLE_MOTOR =
            TalonMotorController(
                can::SWERVE_FRONT_LEFT_ANGLE_ID,
                TalonMotorController::TalonFX
            ).SetInversion(false)
                ->SetCurrentLimit(25)
                ->SetPID(0.3, 0, 0);
        static frc::Translation2d BACK_RIGHT_MODULE_TRANSLATION(
            -0.3175_m,
            -0.2413_m
        );
    }
}