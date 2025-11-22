package frc.robot.subsystems.climb;

import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.util.SubsystemDataProcessor;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO extends SubsystemDataProcessor.IODataRefresher {

    default void setClimberIntakeVoltage(double climberVoltage) {}

    default void resetCarriagePositionInMeters(double carriagePosition) {}

    @AutoLog
    class ClimberIOInputs {
        public double climberRollerVoltage = 0.0;
        public double climberRollerSupplyCurrent = 0.0;
        public double climberRollerStatorCurrent = 0.0;
        public double climberRollerTemperature = 0.0;
        public double climberVelocityRPS = 0.0;
        public boolean climberRatchetEngaged = false;
    }

    default void updateInputs(ClimberIOInputs inputs) {}

    default void setDutyCycleOutputForCarriage(double dutyCycleOutput) {}

    default void setCarriageTargetExtensionInMeters(double desiredExtension) {}

    default void setNeutralMode(NeutralModeValue neutralMode) {}

    @Override
    default void refreshData() {}
}
