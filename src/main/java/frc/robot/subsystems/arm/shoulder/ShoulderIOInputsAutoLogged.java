package frc.robot.subsystems.arm.shoulder;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import frc.robot.subsystems.arm.shoulder.ShoulderIO.ShoulderIOInputs;

public class ShoulderIOInputsAutoLogged extends ShoulderIOInputs implements LoggableInputs {

	@Override
	public void toLog(LogTable table) {
		table.put("shoulderAngleDegrees", shoulderAngle.getDegrees());
        table.put("shoulderAppliedVolts", shoulderAppliedVolts);
        table.put("shoulderSupplyCurrentAmps", shoulderSupplyCurrentAmps);
        table.put("shoulderStatorCurrentAmps", shoulderStatorCurrentAmps);
        table.put("shoulderAngularVelocityRadPerSec", shoulderAngularVelocityRadPerSec);
        table.put("shoulderAngularAccelerationRadPerSecSquared", shoulderAngularAccelerationRadPerSecSquared);
        table.put("shoulderMotorTemp", shoulderMotorTemp);
	}

	@Override
	public void fromLog(LogTable table) {
		shoulderAngle                                  = shoulderAngle.fromDegrees(table.get("shoulderAngleDegrees", shoulderAngle.getDegrees()));
        shoulderAppliedVolts                           = table.get("shoulderAppliedVolts", shoulderAppliedVolts);
        shoulderSupplyCurrentAmps                      = table.get("shoulderSupplyCurrentAmps", shoulderSupplyCurrentAmps);
        shoulderStatorCurrentAmps                      = table.get("shoulderStatorCurrentAmps", shoulderStatorCurrentAmps);
        shoulderAngularVelocityRadPerSec               = table.get("shoulderAngularVelocityRadPerSec", shoulderAngularVelocityRadPerSec);
        shoulderAngularAccelerationRadPerSecSquared    = table.get("shoulderAngularAccelerationRadPerSecSquared", shoulderAngularAccelerationRadPerSecSquared);
        shoulderMotorTemp                              = table.get("shoulderMotorTemp", shoulderMotorTemp);
	}

}
