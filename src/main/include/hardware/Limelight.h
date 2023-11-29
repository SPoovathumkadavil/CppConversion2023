
#pragma once

#include <frc/geometry/Pose2d.h>
#include <networktables/NetworkTable.h>
#include "networktables/NetworkTableInstance.inc"

class Limelight
{
    
public:
    Limelight(std::string limelightName)
    {
        table = nt::NetworkTableInstance::GetDefault().GetTable(limelightName);
    }

    void setPipeline(int index)
    {
        setEntry("pipeline", index);
    }

    frc::Pose2d getRobotPoseToField()
    {
        std::vector<double> raw = table.get()->GetEntry("botpose").GetDoubleArray(def);
        return frc::Pose2d(
            units::meter_t{raw[0]},
            units::meter_t{raw[1]},
            frc::Rotation2d(units::degree_t{raw[5]})
        );
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