package frc.robot.subsystems.hpintake;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.util.SubsystemDataProcessor;

public interface HPIntakeIO extends SubsystemDataProcessor.IODataRefresher {

    default void updateInputs(IntakeIOInputs inputs) {}

    @AutoLog
    class IntakeIOInputs {
        public double  motorVoltage                     = 0.0;
        public double  motorSupplyCurrent               = 0.0;
        public double  motorStatorCurrent               = 0.0;
        public double  motorTemperature                 = 0.0;
        public double  motorVelocityRPS                 = 0.0;
        public boolean isTableCANTRangeTripped          = false;
        public boolean isTableDropped                   = false;
        public double  tableCANRangeDistanceInMetters   = 0.0;
        public double  servoDistanceInMetters           = 0.0;
    }

    default void setMotorVoltage(double volts) {}

    default void setServoPosition(double positionMeters) {}

    @Override
    default void refreshData() {}
}