package frc.robot.subsystems.arm;

import frc.robot.constants.ArmPoseConstants;
import frc.robot.constants.Constants.ArmConstants;
import frc.robot.subsystems.BaseSubsystem;
import frc.robot.subsystems.arm.shoulder.ShoulderIO;
import frc.robot.subsystems.arm.shoulder.ShoulderIOInputsAutoLogged;
import frc.robot.subsystems.arm.shoulder.ShoulderIO.ShoulderIOInputs;
import frc.robot.subsystems.arm.wrist.WristIO;
import frc.robot.subsystems.arm.wrist.WristIOInputsAutoLogged;
import frc.robot.subsystems.arm.wrist.WristIO.WristIOInputs;
import frc.robot.util.ArmPosition;
import frc.robot.util.SubsystemDataProcessor;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

    private ArmPosition wantedArmPose;

    private final ShoulderIO shoulderIO;
    private final WristIO wristIO;

    private final ShoulderIOInputsAutoLogged shoulderInputs = new ShoulderIOInputsAutoLogged();
    private final WristIOInputsAutoLogged wristInputs = new WristIOInputsAutoLogged();

    private double shoulderHomeTimeStamp = Double.NaN;
    private double wristHomeTimeStamp = Double.NaN;
    private boolean isShoulderHomed = false;
    private boolean isWristHomed = false;

    private boolean hasInitialHomeCompleted = false;

    public enum WantedState {
        HOME,
        IDLE,
        MOVE_TO_POSITION,
    }

    private enum SystemState {
        HOMING_SHOULDER,
        HOMING_WRIST,
        IDLING,
        MOVING_TO_POSITION
    }

    private WantedState wantedState = WantedState.IDLE;
    private WantedState previousWantedState = WantedState.IDLE;
    private SystemState systemState = SystemState.IDLING;

    public ArmSubsystem(ShoulderIO shoulderIO, WristIO wristIO) {
        this.shoulderIO = shoulderIO;
        this.wristIO = wristIO;

        wantedArmPose = ArmPoseConstants.ZEROED;

        SubsystemDataProcessor.createAndStartSubsystemDataProcessor(
                () -> {
                        synchronized (shoulderInputs) {
                            synchronized (wristInputs) {
                                shoulderIO.updateInputs(shoulderInputs);
                                wristIO.updateInputs(wristInputs);
                            }
                        }
                },
                shoulderIO,
                wristIO);
    }

    @Override
    public void periodic() {
        synchronized (shoulderInputs) {
            synchronized (wristInputs) {
                Logger.processInputs("Subsystems/Arm/Shoulder", shoulderInputs);
                Logger.processInputs("Subsystems/Arm/Wrist", wristInputs);

                systemState = handleStateTransitions();

                Logger.recordOutput("Subsystems/Arm/SystemState", systemState);
                Logger.recordOutput("Subsystems/Arm/WantedState", wantedState);
                Logger.recordOutput("Subsystems/Arm/ReachedSetpoint", reachedSetpoint());

                Rotation2d wantedShoulderAngle;
                Rotation2d wantedWristAngle;

                if (wantedArmPose != null) {
                    wantedShoulderAngle = wantedArmPose.getShoulderAngleRot2d();
                    wantedWristAngle = wantedArmPose.getWristAngleRot2d();
                    Logger.recordOutput("Subsystems/Arm/WantedShoulderAngle", wantedShoulderAngle);
                    Logger.recordOutput("Subsystems/Arm/WantedWristAngle", wantedWristAngle);
                }

                applyStates();

                previousWantedState = this.wantedState;
            }
        }
    }

    public SystemState handleStateTransitions() {
        switch (wantedState) {
            case HOME:
                if (previousWantedState != WantedState.HOME) {
                    isShoulderHomed = false;
                    isWristHomed = false;
                }

                if (!DriverStation.isDisabled()) {
                    if (Math.abs(shoulderInputs.shoulderAngularVelocityRadPerSec)
                            <= ArmConstants.SHOULDER_ZERO_VELOCITY_THRESHOLD_RADIANS_PER_SECOND) {
                        if (Double.isNaN(shoulderHomeTimeStamp)) {
                            shoulderHomeTimeStamp = Timer.getFPGATimestamp();
                            return SystemState.HOMING_SHOULDER;
                        } else if ((Timer.getFPGATimestamp() - shoulderHomeTimeStamp)
                                >= ArmConstants.ZERO_VELOCITY_TIME_PERIOD) {

                            if (!hasInitialHomeCompleted) {
                                hasInitialHomeCompleted = true;
                                tarShoulder();
                                return SystemState.HOMING_WRIST;
                            }

                            if (Math.abs(wristInputs.wristAngularVelocityRadPerSec)
                                    <= ArmConstants.WRIST_ZERO_VELOCITY_THRESHOLD_RADIANS_PER_SECOND) {
                                if (Double.isNaN(wristHomeTimeStamp)) {
                                    wristHomeTimeStamp = Timer.getFPGATimestamp();
                                    return SystemState.HOMING_WRIST;
                                } else if ((Timer.getFPGATimestamp() - wristHomeTimeStamp)
                                        >= ArmConstants.ZERO_VELOCITY_TIME_PERIOD) {

                                    isWristHomed = true;
                                    wristHomeTimeStamp = Double.NaN;
                                    shoulderHomeTimeStamp = Double.NaN;

                                    tareWrist();
                                    hasInitialHomeCompleted = false;

                                    setWantedState(WantedState.IDLE);
                                    return SystemState.IDLING;

                                } else {
                                    return SystemState.HOMING_WRIST;
                                }
                            } else {
                                wristHomeTimeStamp = Double.NaN;
                                return SystemState.HOMING_WRIST;
                            }
                        } else {
                            return SystemState.HOMING_SHOULDER;
                        }
                    } else {
                        shoulderHomeTimeStamp = Double.NaN;
                        return SystemState.HOMING_SHOULDER;
                    }
                } else {
                    return SystemState.HOMING_SHOULDER;
                }
            case IDLE:
                return SystemState.IDLING;
            case MOVE_TO_POSITION:
                return SystemState.MOVING_TO_POSITION;
        }
        return SystemState.IDLING;
    }

    public void applyStates() {
        switch (systemState) {
            case HOMING_SHOULDER:
                shoulderIO.setDutyCycle(ArmConstants.SHOULDER_ZEROING_DUTY_CYCLE);
                break;
            case HOMING_WRIST:
                wristIO.setDutyCycle(ArmConstants.WRIST_ZEROING_DUTY_CYCLE);
                shoulderIO.setDutyCycle(ArmConstants.SHOULDER_ZEROING_DUTY_CYCLE);
                break;
            case IDLING:
                shoulderIO.setDutyCycle(0);
                wristIO.setDutyCycle(0);
                break;
            case MOVING_TO_POSITION:
                shoulderIO.setTargetAngle(wantedArmPose.getShoulderAngleRot2d());
                wristIO.setTargetAngle(wantedArmPose.getWristAngleRot2d());
            break;
        }
    }

    public void tareAllAxesUsingButtonValues() {
        synchronized (shoulderInputs) {
            synchronized (wristInputs) {
                isShoulderHomed = true;
                isWristHomed = true;
                shoulderIO.resetShoulderAngle(
                        Rotation2d.fromDegrees(ArmConstants.SHOULDER_BUTTON_HOME_ANGLE_DEGREES));
                wristIO.resetWristAngle(Rotation2d.fromDegrees(ArmConstants.WRIST_BUTTON_HOME_ANGLE_DEGREES));
            }
        }
    }

    public void tareAllAxes() {
        tareWrist();
        tarShoulder();
    }

    public void tareWrist() {
        synchronized (wristInputs) {
            isWristHomed = true;
            wristIO.resetWristAngle(Rotation2d.fromDegrees(ArmConstants.WRIST_DRIVEN_HOME_ANGLE_DEGREES));
        }
    }

    public void tarShoulder() {
        synchronized (shoulderInputs) {
            isShoulderHomed = true;
            shoulderIO.resetShoulderAngle(Rotation2d.fromDegrees(ArmConstants.SHOULDER_DRIVEN_HOME_ANGLE_DEGREES));
        }
    }


    public void setNeutralMode(NeutralModeValue neutralModeValue) {
        synchronized (shoulderInputs) {
            synchronized (wristInputs) {
                shoulderIO.setNeutralMode(neutralModeValue);
                wristIO.setNeutralMode(neutralModeValue);
            }
        }
    }

    public boolean hasHomeCompleted() {
        return isShoulderHomed && isWristHomed;
    }

    public Rotation2d getCurrentShoulderPosition() {
        synchronized (shoulderInputs) {
            return shoulderInputs.shoulderAngle;
        }
    }

    public Rotation2d getCurrentWristPosition() {
        synchronized (wristInputs) {
            return wristInputs.wristAngle;
        }
    }

    public ArmPosition getWantedArmPose() {
        return wantedArmPose;
    }

    public boolean reachedSetpoint() {
        synchronized (shoulderInputs) {
            synchronized (wristInputs) {
                return MathUtil.isNear(
                            wantedArmPose.getShoulderAngleRot2d().getDegrees(),
                            shoulderInputs.shoulderAngle.getDegrees(),
                            ArmConstants.SHOULDER_SETPOINT_TOLERANCE_DEGREES)
                        && MathUtil.isNear(
                            wantedArmPose.getWristAngleRot2d().getDegrees(),
                            wristInputs.wristAngle.getDegrees(),
                            ArmConstants.WRIST_SETPOINT_TOLERANCE_DEGREES);
            }
        }
    }

    public boolean reachedSetpoint(ArmPosition armPosition) {
        synchronized (shoulderInputs) {
            synchronized (wristInputs) {
                return MathUtil.isNear(
                                armPosition.getShoulderAngleRot2d().getDegrees(),
                                shoulderInputs.shoulderAngle.getDegrees(),
                                ArmConstants.SHOULDER_SETPOINT_TOLERANCE_DEGREES)
                        && MathUtil.isNear(
                                armPosition.getWristAngleRot2d().getDegrees(),
                                wristInputs.wristAngle.getDegrees(),
                                ArmConstants.WRIST_SETPOINT_TOLERANCE_DEGREES);
            }
        }
    }

    public void setWantedState(WantedState wantedState) {
        this.wantedState = wantedState;
    }

    public void setWantedState(WantedState wantedState, ArmPosition armPosition) {
        this.wantedState = wantedState;
        this.wantedArmPose = armPosition;
    }

    public void setOnlyExtensionAndShoulder(WantedState wantedState, ArmPosition armPosition) {
        this.wantedState = wantedState;
        var wantedWrist = wantedArmPose.getWristAngleRot2d();
        this.wantedArmPose = new ArmPosition(
                armPosition.getShoulderAngleRot2d(), wantedWrist);
    }
}