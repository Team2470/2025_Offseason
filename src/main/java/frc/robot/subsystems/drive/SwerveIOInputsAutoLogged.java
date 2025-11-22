package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import frc.robot.subsystems.drive.SwerveIO.SwerveIOInputs;

public class SwerveIOInputsAutoLogged extends SwerveIOInputs implements LoggableInputs{

    @Override
    public void toLog(LogTable table) {
        table.put("Pose", Pose);
        table.put("Speeds", Speeds);
        table.put("ModuleStates", ModuleStates);
        table.put("ModuleTargets", ModuleTargets);
        table.put("ModulePositions", ModulePositions);
        table.put("RawHeading", RawHeading);
        table.put("Timestamp", Timestamp);
        table.put("OdometryPeriod", OdometryPeriod);
        table.put("SuccessfulDaqs", SuccessfulDaqs);
        table.put("FailedDaqs", FailedDaqs);
    }

    @Override
    public void fromLog(LogTable table) {
        Pose            = table.get("Pose", Pose);
        Speeds          = table.get("Speeds", Speeds);
        ModuleStates    = table.get("ModuleStates", ModuleStates);
        ModuleTargets   = table.get("ModuleTargets", ModuleTargets);
        ModulePositions = table.get("ModulePositions", ModulePositions);
        RawHeading      = table.get("RawHeading", RawHeading);
        Timestamp       = table.get("Timestamp", Timestamp);
        OdometryPeriod  = table.get("OdometryPeriod", OdometryPeriod);
        SuccessfulDaqs  = table.get("SuccessfulDaqs", SuccessfulDaqs);
        FailedDaqs      = table.get("FailedDaqs", FailedDaqs);
    }

}
