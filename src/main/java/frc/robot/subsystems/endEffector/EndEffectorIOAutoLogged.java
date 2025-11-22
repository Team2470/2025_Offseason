package frc.robot.subsystems.endEffector;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import frc.robot.subsystems.endEffector.EndEffectorIO.EndEffectorIOInputs;

public class EndEffectorIOAutoLogged extends EndEffectorIOInputs implements LoggableInputs{

    @Override
    public void toLog(LogTable table) {
        table.put("endEffectorCoralAppliedVolts", endEffectorCoralAppliedVolts);
        table.put("endEffectorCoralSupplyCurrentAmps", endEffectorCoralSupplyCurrentAmps);
        table.put("endEffectorCoralStatorCurrentAmps", endEffectorCoralStatorCurrentAmps);
        table.put("endEffectorCoralVelocityMetersPerSec", endEffectorCoralVelocityMetersPerSec);
        table.put("endEffectorCoralAccelerationMetersPerSecSquared", endEffectorCoralAccelerationMetersPerSecSquared);

        table.put("endEffectorAlgaeAppliedVolts", endEffectorAlgaeAppliedVolts);
        table.put("endEffectorAlgaeSupplyCurrentAmps", endEffectorAlgaeSupplyCurrentAmps);
        table.put("endEffectorAlgaeStatorCurrentAmps", endEffectorAlgaeStatorCurrentAmps);
        table.put("endEffectorAlgaeVelocityMetersPerSec", endEffectorAlgaeVelocityMetersPerSec);
        table.put("endEffectorAlgaeAccelerationMetersPerSecSquared", endEffectorAlgaeAccelerationMetersPerSecSquared);

        table.put("endEffectorCoralMotorTemp", endEffectorCoralMotorTemp);
        table.put("endEffectorAlgaeMotorTemp", endEffectorAlgaeMotorTemp);

        table.put("hasCoral", hasCoral);
        table.put("hasAlgae", hasAlgae);
    }

    @Override
    public void fromLog(LogTable table) {
        endEffectorCoralAppliedVolts                    = table.get("endEffectorCoralAppliedVolts", endEffectorCoralAppliedVolts);
        endEffectorCoralSupplyCurrentAmps               = table.get("endEffectorCoralSupplyCurrentAmps", endEffectorCoralSupplyCurrentAmps);
        endEffectorCoralStatorCurrentAmps               = table.get("endEffectorCoralStatorCurrentAmps", endEffectorCoralStatorCurrentAmps);
        endEffectorCoralVelocityMetersPerSec            = table.get("endEffectorCoralVelocityMetersPerSec", endEffectorCoralVelocityMetersPerSec);
        endEffectorCoralAccelerationMetersPerSecSquared = table.get("endEffectorCoralAccelerationMetersPerSecSquared", endEffectorCoralAccelerationMetersPerSecSquared);

        endEffectorAlgaeAppliedVolts                    = table.get("endEffectorAlgaeAppliedVolts", endEffectorAlgaeAppliedVolts);
        endEffectorAlgaeSupplyCurrentAmps               = table.get("endEffectorAlgaeSupplyCurrentAmps", endEffectorAlgaeSupplyCurrentAmps);
        endEffectorAlgaeStatorCurrentAmps               = table.get("endEffectorAlgaeStatorCurrentAmps", endEffectorAlgaeStatorCurrentAmps);
        endEffectorAlgaeVelocityMetersPerSec            = table.get("endEffectorAlgaeVelocityMetersPerSec", endEffectorAlgaeVelocityMetersPerSec);
        endEffectorAlgaeAccelerationMetersPerSecSquared = table.get("endEffectorAlgaeAccelerationMetersPerSecSquared", endEffectorAlgaeAccelerationMetersPerSecSquared);

        endEffectorCoralMotorTemp                       = table.get("endEffectorCoralMotorTemp", endEffectorCoralMotorTemp);
        endEffectorAlgaeMotorTemp                       = table.get("endEffectorAlgaeMotorTemp", endEffectorAlgaeMotorTemp);

        hasCoral                                        = table.get("hasCoral", hasCoral);
        hasAlgae                                        = table.get("hasAlgae", hasAlgae);
    }

}
