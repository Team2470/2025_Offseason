package frc.robot.subsystems.arm.wrist;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.arm.wrist.WristIO.WristIOInputs;

public class WristIOInputsAutoLogged extends WristIOInputs implements LoggableInputs {

    @Override
    public void toLog(LogTable table) {
        table.put("wristAngleDegrees", wristAngle.getDegrees());
        table.put("wristAppliedVolts", wristAppliedVolts);
        table.put("wristSupplyCurrentAmps", wristSupplyCurrentAmps);
        table.put("wristStatorCurrentAmps", wristStatorCurrentAmps);
        table.put("wristAngularVelocityRadPerSec", wristAngularVelocityRadPerSec);
        table.put("wristAngularAccelerationRadPerSecSquared", wristAngularAccelerationRadPerSecSquared);
        table.put("wristMotorTemp", wristMotorTemp);
    }

    @Override
    public void fromLog(LogTable table) {
        wristAngle                                  = Rotation2d.fromDegrees(table.get("wristAngleDegrees", wristAngle.getDegrees()));
        wristAppliedVolts                           = table.get("wristAppliedVolts", wristAppliedVolts);
        wristSupplyCurrentAmps                      = table.get("wristSupplyCurrentAmps", wristSupplyCurrentAmps);
        wristStatorCurrentAmps                      = table.get("wristStatorCurrentAmps", wristStatorCurrentAmps);
        wristAngularVelocityRadPerSec               = table.get("wristAngularVelocityRadPerSec", wristAngularVelocityRadPerSec);
        wristAngularAccelerationRadPerSecSquared    = table.get("wristAngularAccelerationRadPerSecSquared", wristAngularAccelerationRadPerSecSquared);
        wristMotorTemp                              = table.get("wristMotorTemp", wristMotorTemp);
    }

}
