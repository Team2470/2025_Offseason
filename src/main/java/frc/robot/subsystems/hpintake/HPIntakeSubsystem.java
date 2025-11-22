package frc.robot.subsystems.hpintake;

import frc.robot.subsystems.BaseSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

// Climb example
public class HPIntakeSubsystem extends BaseSubsystem {
    private final MotorController motor; // replace with your motor class
    private double goalMeters = 0.0;
    private final PIDController pid = new PIDController(1.0, 0, 0);

    public HPIntakeSubsystem(MotorController motor) {
        this.motor = motor;
    }

    public void setGoalMeters(double meters) {
        this.goalMeters = meters;
    }

    public boolean isAtGoal(double toleranceMeters) {
        double err = Math.abs(getPositionMeters() - goalMeters);
        return err <= toleranceMeters;
    }

    @Override
    public void periodic() {
        double output = pid.calculate(getPositionMeters(), goalMeters);
        motor.set(output);
    }

    @Override
    public void stop() {
        motor.set(0);
    }

    public double getPositionMeters() {
        // read encoder and convert
        return 0.0;
    }
}