
#pragma once

#include <frc/geometry/Pose2d.h>
#include <networktables/NetworkTable.h>
#include <frc/DriverStation.h>
#include "networktables/NetworkTableInstance.inc"

class Limelight
{
    
public:
    Limelight(std::string limelightName)
    {
        table = nt::NetworkTableInstance::GetDefault().GetTable(limelightName);
    }

    void SetPipeline(int index)
    {
        setEntry("pipeline", index);
    }

    frc::Pose2d GetRobotPoseToField()
    {
        std::vector<double> raw = table.get()->GetEntry("botpose").GetDoubleArray(def);
        return frc::Pose2d(
            units::meter_t{raw[0]},
            units::meter_t{raw[1]},
            frc::Rotation2d(units::degree_t{raw[5]})
        );
    }

    frc::Pose2d GetRobotPoseToAlliance(frc::DriverStation::Alliance alliance)
    {
        std::vector<double> raw;
        switch(alliance)
        {
            case frc::DriverStation::Alliance::kRed:
                raw = table.get()->GetEntry("botpose_wpired").GetDoubleArray(def);
                break;
            case frc::DriverStation::Alliance::kBlue:
                raw = table.get()->GetEntry("botpose_wpiblue").GetDoubleArray(def);
                break;
            default:
                break;
        }
        return frc::Pose2d(
            units::meter_t{raw[0]},
            units::meter_t{raw[1]},
            frc::Rotation2d(units::degree_t{raw[5]})
        );
    }

    frc::Pose2d GetRobotPoseToTarget()
    {
        std::vector<double> raw = table.get()->GetEntry("botpose_targetspace").GetDoubleArray(def);
        return frc::Pose2d(
            units::meter_t{raw[0]},
            units::meter_t{raw[1]},
            frc::Rotation2d(units::degree_t{raw[5]})
        );
    }

    frc::Pose2d GetTargetPoseToCamera()
    {
        std::vector<double> raw = table.get()->GetEntry("targetpose_cameraspace").GetDoubleArray(def);
        return frc::Pose2d(
            units::meter_t{raw[0]},
            units::meter_t{raw[1]},
            frc::Rotation2d(units::degree_t{raw[5]})
        );
    }

    frc::Pose2d GetTargetPoseToRobot()
    {
        std::vector<double> raw = table.get()->GetEntry("targetpose_robotspace").GetDoubleArray(def);
        return frc::Pose2d(
            units::meter_t{raw[0]},
            units::meter_t{raw[1]},
            frc::Rotation2d(units::degree_t{raw[5]})
        );
    }

    frc::Pose2d GetCameraPoseToTarget()
    {
        std::vector<double> raw = table.get()->GetEntry("camerapose_targetspace").GetDoubleArray(def);
        return frc::Pose2d(
            units::meter_t{raw[0]},
            units::meter_t{raw[1]},
            frc::Rotation2d(units::degree_t{raw[5]})
        );
    }

    int GetTargetTagId()
    {
        return (int) table.get()->GetEntry("tid").GetInteger(0);
    }
    
private:
    std::shared_ptr<nt::NetworkTable> table;
    double def[6]; // A default return value

    double getEntry(std::string key)
    {
        return table.get()->GetEntry(key).GetDouble(0);
    }

    void setEntry(std::string key, int value)
    {
        table.get()->GetEntry(key).SetInteger(value);
    }

    void setEntry(std::string key, double value)
    {
        table.get()->GetEntry(key).SetDouble(value);
    }

    void setEntry(std::string key, float value)
    {
        table.get()->GetEntry(key).SetFloat(value);
    }

    void setEntry(std::string key, bool value)
    {
        table.get()->GetEntry(key).SetBoolean(value);
    }

    void setEntry(std::string key, std::string value)
    {
        table.get()->GetEntry(key).SetString(value);
    }
};