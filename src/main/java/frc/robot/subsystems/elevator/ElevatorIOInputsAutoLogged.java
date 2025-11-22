package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import frc.robot.subsystems.elevator.ElevatorIO.ElevatorIOInputs;

public class ElevatorIOInputsAutoLogged extends ElevatorIOInputs implements LoggableInputs {
    @Override
    public void toLog(LogTable table) {
        table.put("elevatorPositionInMeters", elevatorPositionInMeters);
        table.put("elevatorAppliedVolts", elevatorAppliedVolts);
        table.put("elevatorSupplyCurrentAmps", elevatorSupplyCurrentAmps);
        table.put("elevatorStatorCurrentAmps", elevatorStatorCurrentAmps);
        table.put("elevatorVelocityMetersPerSec", elevatorVelocityMetersPerSec);
        table.put("elevatorAccelerationMetersPerSecSquared", elevatorAccelerationMetersPerSecSquared);
        table.put("elevatorOneMotorTemp", elevatorOneMotorTemp);
        table.put("elevatorTwoMotorTemp", elevatorTwoMotorTemp);
    }
    @Override
    public void fromLog(LogTable table) {
        elevatorPositionInMeters                = table.get("elevatorPositionInMeters", elevatorPositionInMeters);
        elevatorAppliedVolts                    = table.get("elevatorAppliedVolts", elevatorAppliedVolts);
        elevatorSupplyCurrentAmps               = table.get("elevatorSupplyCurrentAmps", elevatorSupplyCurrentAmps);
        elevatorStatorCurrentAmps               = table.get("elevatorStatorCurrentAmps", elevatorStatorCurrentAmps);
        elevatorVelocityMetersPerSec            = table.get("elevatorVelocityMetersPerSec", elevatorVelocityMetersPerSec);
        elevatorAccelerationMetersPerSecSquared = table.get("elevatorAccelerationMetersPerSecSquared", elevatorAccelerationMetersPerSecSquared);
        elevatorOneMotorTemp                    = table.get("elevatorOneMotorTemp", elevatorOneMotorTemp);
        elevatorTwoMotorTemp                    = table.get("elevatorTwoMotorTemp", elevatorTwoMotorTemp);
    }
}
