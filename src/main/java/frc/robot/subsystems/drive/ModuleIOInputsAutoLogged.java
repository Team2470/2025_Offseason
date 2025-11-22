package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import frc.robot.subsystems.drive.SwerveIO.ModuleIOInputs;

public class ModuleIOInputsAutoLogged extends ModuleIOInputs implements LoggableInputs {

    @Override
    public void toLog(LogTable table) {
        table.put("driveSupplyCurrentAmps", driveSupplyCurrentAmps);
        table.put("driveStatorCurrentAmps", driveStatorCurrentAmps);
        table.put("driveAppliedVolts", driveAppliedVolts);
        table.put("driveTemperature", driveTemperature);

        table.put("steerSupplyCurrentAmps", steerSupplyCurrentAmps);
        table.put("steerStatorCurrentAmps", steerStatorCurrentAmps);
        table.put("steerAppliedVolts", steerAppliedVolts);
        table.put("steerTemperature", steerTemperature);
    }

    @Override
    public void fromLog(LogTable table) {
        driveSupplyCurrentAmps  = table.get("driveSupplyCurrentAmps", driveSupplyCurrentAmps);
        driveStatorCurrentAmps  = table.get("driveStatorCurrentAmps", driveStatorCurrentAmps);
        driveAppliedVolts       = table.get("driveAppliedVolts", driveAppliedVolts);
        driveTemperature        = table.get("driveTemperature", driveTemperature);

        steerSupplyCurrentAmps  = table.get("steerSupplyCurrentAmps", steerSupplyCurrentAmps);
        steerStatorCurrentAmps  = table.get("steerStatorCurrentAmps", steerStatorCurrentAmps);
        steerAppliedVolts       = table.get("steerAppliedVolts", steerAppliedVolts);
        steerTemperature        = table.get("steerTemperature", steerTemperature);
    }

}
