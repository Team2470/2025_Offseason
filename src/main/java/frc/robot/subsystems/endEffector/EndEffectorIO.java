package frc.robot.subsystems.endEffector;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.util.SubsystemDataProcessor;

public interface EndEffectorIO extends SubsystemDataProcessor.IODataRefresher{
        default void updateInputs(EndEffectorIOInputs inputs) {}

    @AutoLog
    class EndEffectorIOInputs {
        public double endEffectorCoralAppliedVolts;
        public double endEffectorCoralSupplyCurrentAmps;
        public double endEffectorCoralStatorCurrentAmps;
        public double endEffectorCoralVelocityMetersPerSec;
        public double endEffectorCoralAccelerationMetersPerSecSquared;

        public double endEffectorAlgaeAppliedVolts;
        public double endEffectorAlgaeSupplyCurrentAmps;
        public double endEffectorAlgaeStatorCurrentAmps;
        public double endEffectorAlgaeVelocityMetersPerSec;
        public double endEffectorAlgaeAccelerationMetersPerSecSquared;

        public double endEffectorCoralMotorTemp;
        public double endEffectorAlgaeMotorTemp;

        public boolean hasCoral = false;
        public boolean hasAlgae = false;
    }

    default void setCoralMotorVoltage(double voltage) {}

    default void setAlgaeMotorVoltage(double voltage) {}

    default void resetEndEffectorPosition(double positionInMeters) {}

    default void setDutyCycle(double dutyCycle) {}

    default void setNeutralMode(NeutralModeValue neutralMode) {}

    @Override
    default void refreshData() {}
}
