package frc.robot.subsystems.elevator;

import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.BaseSubsystem;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

// Elevator example
public class ElevatorSubsystem extends BaseSubsystem {
    private final ElevatorIO elevatorIO; // replace with your motor class
    private final ElevatorIOInputsAutoLogged elevatorInputs = new ElevatorIOInputsAutoLogged();
    private double wantedElevatorHeight = ElevatorConstants.ZEROED;
    private final PIDController pid = new PIDController(1.0, 0, 0);

    public enum WantedState {
        HOME,
        IDLE,
        MOVE_TO_POSITION,
    }

    private enum SystemState {
        HOMING_ELEVATOR,
        IDLING,
        MOVING_TO_POSITION
    }

    private WantedState wantedState = WantedState.IDLE;
    private WantedState previousWantedState = WantedState.IDLE;
    private SystemState systemState = SystemState.IDLING;
    private boolean isElevatorHomed = false;

    public ElevatorSubsystem(ElevatorIO elevatorIO) {
        this.elevatorIO = elevatorIO;
    }

    public void setWantedElevatorHeight(double meters) {
        this.wantedElevatorHeight = meters;
    }

    public boolean isAtGoal(double toleranceMeters) {
        double err = Math.abs(getPositionMeters() - wantedElevatorHeight);
        return err <= toleranceMeters;
    }

    @Override
    public void periodic() {
        systemState = handleStateTransitions();
        applyStates();
    }

    public SystemState handleStateTransitions() {
        switch (wantedState) {
            case HOME:
                if (previousWantedState != WantedState.HOME) {
                    isElevatorHomed = false;
                }
                if(!isElevatorHomed) {
                    tareElevator();
                    return SystemState.HOMING_ELEVATOR;
                } else {
                    return SystemState.IDLING;
                }
            case IDLE:
                return SystemState.IDLING;
            case MOVE_TO_POSITION:
                return SystemState.MOVING_TO_POSITION;
            default:
                return SystemState.IDLING;
        }
    }

    public void applyStates() {
        switch (systemState) {
            case HOMING_ELEVATOR:
                elevatorIO.setDutyCycle(ElevatorConstants.ELEVATOR_ZEROING_DUTY_CYCLE); // Move down slowly to find home
                break;
            case IDLING:
                elevatorIO.setDutyCycle(0); // Hold position
                break;
            case MOVING_TO_POSITION:
                if(isElevatorHomed) {
                    elevatorIO.setTargetElevator(wantedElevatorHeight);
                }
                break;
        }
        previousWantedState = wantedState;
    }

    public void tareElevator(){
        elevatorIO.resetElevatorPosition(ElevatorConstants.ZEROED);
        isElevatorHomed = true;
    }

    @Override
    public void stop() {
    }

    public WantedState getWantedState() {
        return wantedState;
    }

    public double getWantedElevatorHeight() {
        return wantedElevatorHeight;
    }

    public void setWantedState(WantedState wantedState) {
        this.wantedState = wantedState;
    }

    public void setWantedState(WantedState wantedState, double wantedElevatorHeight) {
        this.wantedState = wantedState;
        this.wantedElevatorHeight = wantedElevatorHeight;
    }

    public double getPositionMeters() {
        // read encoder and convert
        return elevatorInputs.elevatorPositionInMeters;
    }

    public boolean reachedSetHeight(double height) {
        return MathUtil.isNear(
                height,
                getPositionMeters(),
                ElevatorConstants.ELEVATOR_SETPOINT_TOLERANCE_METERS);
    }

    public boolean reachedSetHeight() {
        return MathUtil.isNear(
                wantedElevatorHeight,
                getPositionMeters(),
                ElevatorConstants.ELEVATOR_SETPOINT_TOLERANCE_METERS);
    }

    public void setNeutralMode (NeutralModeValue neutralMode) {
        elevatorIO.setNeutralMode(neutralMode);
    }

    public boolean hasHomeCompleted() {
        return isElevatorHomed;
    }
}
