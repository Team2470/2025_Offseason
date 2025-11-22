package frc.robot.subsystems.arm.shoulder;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.SubsystemDataProcessor;

public interface ShoulderIO extends SubsystemDataProcessor.IODataRefresher {
    default void updateInputs(ShoulderIOInputs inputs) {}

    @AutoLog
    class ShoulderIOInputs {
        public Rotation2d shoulderAngle = Rotation2d.kZero;
        public double shoulderAppliedVolts;
        public double shoulderSupplyCurrentAmps;
        public double shoulderStatorCurrentAmps;
        public double shoulderAngularVelocityRadPerSec;
        public double shoulderAngularAccelerationRadPerSecSquared;
        public double shoulderMotorTemp;
    }

    default void setTargetAngle(Rotation2d target) {}

    default void resetShoulderAngle(Rotation2d angle) {}

    default void setDutyCycle(double dutyCycle) {}

    default void setNeutralMode(NeutralModeValue neutralMode) {}

    @Override
    default void refreshData() {}
}
