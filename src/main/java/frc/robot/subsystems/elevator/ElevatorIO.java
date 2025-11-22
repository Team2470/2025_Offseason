package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.util.SubsystemDataProcessor;

public interface ElevatorIO extends SubsystemDataProcessor.IODataRefresher{
        default void updateInputs(ElevatorIOInputs inputs) {}

    @AutoLog
    class ElevatorIOInputs {
        public double elevatorPositionInMeters;

        public double elevatorAppliedVolts;
        public double elevatorSupplyCurrentAmps;
        public double elevatorStatorCurrentAmps;
        public double elevatorVelocityMetersPerSec;
        public double elevatorAccelerationMetersPerSecSquared;

        public double elevatorOneMotorTemp;
        public double elevatorTwoMotorTemp;
    }

    default void setTargetElevator(double positionInMeters) {}

    default void resetElevatorPosition(double positionInMeters) {}

    default void setDutyCycle(double dutyCycle) {}

    default void setNeutralMode(NeutralModeValue neutralMode) {}

    @Override
    default void refreshData() {}
}
