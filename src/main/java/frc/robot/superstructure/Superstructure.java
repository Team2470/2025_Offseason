package frc.robot.superstructure;

import java.util.Scanner;

import frc.robot.RobotState;
import frc.robot.constants.ArmPoseConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.Constants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.arm.ArmSubsystem;
// import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.drive.SwerveSubsystem.WantedState;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.endEffector.EndEffectorIO;
import frc.robot.subsystems.endEffector.EndEffectorSubsystem;
// import frc.robot.subsystems.hpintake.HPIntakeSubsystem;
import frc.robot.util.ArmPosition;

public class Superstructure {
    private final SwerveSubsystem swerveSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final ArmSubsystem armSubsystem;
    private final EndEffectorSubsystem endEffectorSubsystem;
    // private final ClimbSubsystem climbSubsystem;
    // private final HPIntakeSubsystem hpIntakeSubsystem;

    private static final double REGULAR_TELEOP_TRANSLATION_COEFFICIENT = 1.0;

    public enum WantedSuperState {
        HOME,
        HP_INAKE,
        CLIMB_SETUP,
        CLIMB_PULL,
        L1,
        LEFT_L2,
        LEFT_L3,
        LEFT_L4,
        RIGHT_L2,
        RIGHT_L3,
        RIGHT_L4,
        SCORE,
        DRIVE,
        PROCESSOR,
        NET,
        GROUND_ALGAE,
        REEF_ALGAE
    }

    public enum CurrentSuperState {
        HOME,
        HP_INAKE,
        CLIMB_SETUP,
        CLIMB_PULL,
        L1,
        LEFT_L2,
        LEFT_L3,
        LEFT_L4,
        RIGHT_L2,
        RIGHT_L3,
        RIGHT_L4,
        SCORE,
        DRIVE,
        PROCESSOR,
        NET,
        GROUND_ALGAE,
        REEF_ALGAE
    }

    private WantedSuperState wantedSuperState = WantedSuperState.DRIVE;
    private CurrentSuperState currentSuperState = CurrentSuperState.DRIVE;
    private CurrentSuperState previousSuperState;

    public Superstructure(  SwerveSubsystem swerveSubsystem,
                            ElevatorSubsystem elevatorSubsystem,
                            ArmSubsystem armSubsystem,
                            EndEffectorSubsystem endEffectorSubsystem
                            // ClimbSubsystem climbSubsystem,
                            /*HPIntakeSubsystem hpIntakeSubsystem*/) {
        this.swerveSubsystem        = swerveSubsystem;
        this.elevatorSubsystem      = elevatorSubsystem;
        this.armSubsystem           = armSubsystem;
        this.endEffectorSubsystem   = endEffectorSubsystem;
        // this.climbSubsystem     = climbSubsystem;
        // this.hpIntakeSubsystem  = hpIntakeSubsystem;
    }

    public void periodic() {
        // TODO: Update inputs
        currentSuperState = handleStateTransitions();
        applyStates();
    }

    private CurrentSuperState handleStateTransitions() {
        previousSuperState = currentSuperState;
        switch (wantedSuperState) {
            case HOME:
                currentSuperState = CurrentSuperState.HOME;
                break;
            case HP_INAKE:
                currentSuperState = CurrentSuperState.HP_INAKE;
                break;
            case CLIMB_SETUP:
                currentSuperState = CurrentSuperState.CLIMB_SETUP;
                break;
            case CLIMB_PULL:
                currentSuperState = CurrentSuperState.CLIMB_PULL;
                break;
            case L1:
                currentSuperState = CurrentSuperState.L1;
                break;
            case LEFT_L2:
                currentSuperState = CurrentSuperState.LEFT_L2;
                break;
            case LEFT_L3:
                currentSuperState = CurrentSuperState.LEFT_L3;
                break;
            case LEFT_L4:
                currentSuperState = CurrentSuperState.LEFT_L4;
                break;
            case RIGHT_L2:
                currentSuperState = CurrentSuperState.RIGHT_L2;
                break;
            case RIGHT_L3:
                currentSuperState = CurrentSuperState.RIGHT_L3;
                break;
            case RIGHT_L4:
                currentSuperState = CurrentSuperState.RIGHT_L4;
                break;
            case SCORE:
                currentSuperState = CurrentSuperState.SCORE;
                break;
            case DRIVE:
                currentSuperState = CurrentSuperState.DRIVE;
                break;
            case PROCESSOR:
                currentSuperState = CurrentSuperState.PROCESSOR;
                break;
            case NET:
                currentSuperState = CurrentSuperState.NET;
                break;
            case GROUND_ALGAE:
                currentSuperState = CurrentSuperState.GROUND_ALGAE;
                break;
            case REEF_ALGAE:
                currentSuperState = CurrentSuperState.REEF_ALGAE;
                break;
            default:
                currentSuperState = CurrentSuperState.DRIVE;
                break;
        }
        return currentSuperState;
    }

    private void applyStates() {
        // TODO: create logic checks if needed

        switch (currentSuperState){
            case HOME:
                home();
                break;
            case HP_INAKE:
                hp_intake();
                break;
            case CLIMB_SETUP:
                climb_setup();
                break;
            case CLIMB_PULL:
                climb_pull();
                break;
            case L1:
                l1();
                break;
            case LEFT_L2:
                l2(Constants.SuperstructureConstants.ScoringSide.LEFT);
                break;
            case LEFT_L3:
                l3(Constants.SuperstructureConstants.ScoringSide.LEFT);
                break;
            case LEFT_L4:
                l4(Constants.SuperstructureConstants.ScoringSide.LEFT);
                break;
            case RIGHT_L2:
                l2(Constants.SuperstructureConstants.ScoringSide.RIGHT);
                break;
            case RIGHT_L3:
                l3(Constants.SuperstructureConstants.ScoringSide.RIGHT);
                break;
            case RIGHT_L4:
                l4(Constants.SuperstructureConstants.ScoringSide.RIGHT);
                break;
            case SCORE:
                score();
                break;
            case DRIVE:
                drive();
                break;
            case PROCESSOR:
                processor();
                break;
            case NET:
                net();
                break;
            case GROUND_ALGAE:
                ground_algae();
                break;
            case REEF_ALGAE:
                // reef_algae();
                break;
        }
    }

    private void home() {
        if(armSubsystem.hasHomeCompleted()){
            armSubsystem.setWantedState(ArmSubsystem.WantedState.IDLE);
        } else {
            armSubsystem.setWantedState(ArmSubsystem.WantedState.IDLE);
        }
        if(elevatorSubsystem.hasHomeCompleted()){
            elevatorSubsystem.setWantedState(ElevatorSubsystem.WantedState.IDLE);
        } else {
            elevatorSubsystem.setWantedState(ElevatorSubsystem.WantedState.HOME);
        }
    }

    private void hp_intake() {
        if(armSubsystem.reachedSetpoint(ArmPoseConstants.HP_INTAKE_START)) {
            armSubsystem.setWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmPoseConstants.HP_INTAKE_HOLD);
        } else {
            armSubsystem.setWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmPoseConstants.HP_INTAKE_START);
        }
        elevatorSubsystem.setWantedState(ElevatorSubsystem.WantedState.MOVE_TO_POSITION, ElevatorConstants.HP_INTAKE);
        if(armSubsystem.reachedSetpoint(ArmPoseConstants.HP_INTAKE_HOLD)) {
            // hpIntakeSubsystem.setWantedState(HPIntakeSubsystem.WantedState.INTAKE);
        } else {
            // hpIntakeSubsystem.setWantedState(HPIntakeSubsystem.WantedState.STOP);
        }

        // if(BEAM_BREAK_END_EFFECTOR_TRIGGERED) {
        //      armSubsystem.setWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmPoseConstants.DRIVE);
        // } else {
        //     armSubsystem.setWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmPoseConstants.HP_INTAKE_HOLD);
        // }
        swerveSubsystem.setWantedState(WantedState.TELEOP_DRIVE);
    }

    private void climb_setup() {
        // TODO: Add climb subsystem actions
        // Step 1: Extend Climber out
        // Step 2: Drop intake table
        armSubsystem.setWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmPoseConstants.CLIMB);
        elevatorSubsystem.setWantedState(ElevatorSubsystem.WantedState.MOVE_TO_POSITION, ElevatorConstants.CLIMB);
        swerveSubsystem.setWantedState(WantedState.TELEOP_DRIVE);
    }

    private void climb_pull() {
        // TODO: Bring arm in
        armSubsystem.setWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmPoseConstants.CLIMB);
        elevatorSubsystem.setWantedState(ElevatorSubsystem.WantedState.MOVE_TO_POSITION, ElevatorConstants.CLIMB);
        swerveSubsystem.setWantedState(WantedState.TELEOP_DRIVE);
    }

    private void l1() {
        armSubsystem.setWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmPoseConstants.L1);
        elevatorSubsystem.setWantedState(ElevatorSubsystem.WantedState.MOVE_TO_POSITION, ElevatorConstants.L1_CORAL);
        if(isReadyToEjectInTeleopPeriod()) {
            score();
        }
    }

    private void l2(Constants.SuperstructureConstants.ScoringSide side) {
        armSubsystem.setWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmPoseConstants.L2);
        elevatorSubsystem.setWantedState(ElevatorSubsystem.WantedState.MOVE_TO_POSITION, ElevatorConstants.L2_CORAL);
        if(isReadyToEjectInTeleopPeriod()) {
            score();
        }
    }

    private void l3(Constants.SuperstructureConstants.ScoringSide side) {
        armSubsystem.setWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmPoseConstants.L3);
        elevatorSubsystem.setWantedState(ElevatorSubsystem.WantedState.MOVE_TO_POSITION, ElevatorConstants.L3_CORAL);
        if(isReadyToEjectInTeleopPeriod()) {
            score();
        }
    }

    private void l4(Constants.SuperstructureConstants.ScoringSide side) {
        armSubsystem.setWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmPoseConstants.L4);
        elevatorSubsystem.setWantedState(ElevatorSubsystem.WantedState.MOVE_TO_POSITION, ElevatorConstants.L4_CORAL);
        if(isReadyToEjectInTeleopPeriod()) {
            score();
        }
    }

    private void score() {
        swerveSubsystem.setTeleopVelocityCoefficient(0.0);
        if( elevatorSubsystem.getWantedElevatorHeight() == ElevatorConstants.L2_CORAL ||
            elevatorSubsystem.getWantedElevatorHeight() == ElevatorConstants.L3_CORAL ||
            elevatorSubsystem.getWantedElevatorHeight() == ElevatorConstants.L4_CORAL) {
            endEffectorSubsystem.setWantedState(EndEffectorSubsystem.WantedState.OUTTAKE_CORAL);
        } else if (elevatorSubsystem.getWantedElevatorHeight() == ElevatorConstants.L1_CORAL) {
            endEffectorSubsystem.setWantedState(EndEffectorSubsystem.WantedState.OUTTAKE_CORAL_L1);
        } else {
            endEffectorSubsystem.setWantedState(EndEffectorSubsystem.WantedState.OUTTAKE_ALGAE);
        }
    }

    private void drive() {
        armSubsystem.setWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmPoseConstants.DRIVE);
        elevatorSubsystem.setWantedState(ElevatorSubsystem.WantedState.MOVE_TO_POSITION, ElevatorConstants.DRIVE);
        if(elevatorSubsystem.getPositionMeters() > ElevatorConstants.ELEVATOR_HIGH_THRESHOLD) {
            // swerveSubsystem.setTeleopVelocityCoefficient(Constants.SwerveConstants.kTeleopVelocityCoefficientHighElevator);
        } else {
            // swerveSubsystem.setTeleopVelocityCoefficient(Constants.SwerveConstants.kTeleopVelocityCoefficientLowElevator);
        }
    }

    private void processor() {
        armSubsystem.setWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmPoseConstants.PROCESSOR);
        elevatorSubsystem.setWantedState(ElevatorSubsystem.WantedState.MOVE_TO_POSITION, ElevatorConstants.PROCESSOR);
        if(isReadyToEjectInTeleopPeriod()){
            score();
        }
    }

    private void net() {
        armSubsystem.setWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmPoseConstants.NET);
        elevatorSubsystem.setWantedState(ElevatorSubsystem.WantedState.MOVE_TO_POSITION, ElevatorConstants.NET);
        if(isReadyToEjectInTeleopPeriod()){
            score();
        }
    }

    private void ground_algae() {
        armSubsystem.setWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmPoseConstants.GROUND_ALGAE);
        elevatorSubsystem.setWantedState(ElevatorSubsystem.WantedState.MOVE_TO_POSITION, ElevatorConstants.GROUND_ALGAE);
        // INTAKE ALGAE
    }

    private void reef_algae() {
        var levelMap = FieldConstants.isBlueAlliance()
                ? Constants.ReefConstants.blueAllianceAlgae
                : Constants.ReefConstants.redAllianceAlgae;
        var location = levelMap.get(RobotState.getInstance().getClosest60Degrees());
        var targetElevatorHeight = ElevatorConstants.L2_ALGAE;

        if (location.ID == Constants.ReefConstants.AlgaeIntakeLocation.L2) {
            targetElevatorHeight = ElevatorConstants.L2_ALGAE;
        } else {
            targetElevatorHeight = ElevatorConstants.L3_ALGAE;
        }

        armSubsystem.setWantedState(ArmSubsystem.WantedState.MOVE_TO_POSITION, ArmPoseConstants.REEF_ALGAE);
        elevatorSubsystem.setWantedState(ElevatorSubsystem.WantedState.MOVE_TO_POSITION, targetElevatorHeight);
    }

    public boolean isReadyToEjectInTeleopPeriod() {
        return swerveSubsystem.isAtDriveToPointSetpoint()
                && swerveSubsystem.isAtDesiredRotation(Units.degreesToRadians(2.0))
                && armSubsystem.reachedSetpoint();
    }

    public Command setStateCommand(WantedSuperState superState) {
        return setStateCommand(superState);
    }

    public boolean hasCoral() {
        return endEffectorSubsystem.hasCoral();
    }

    public boolean hasAlgae() {
        return endEffectorSubsystem.hasAlgae();
    }

    public Command configureButtonBindings(
        WantedSuperState hasCoralCondition,
        WantedSuperState hasAlgaeCondition,
        WantedSuperState noGamePieceCondition
    ) {
        return  Commands.either(
                    Commands.either(
                        setStateCommand(hasCoralCondition),
                        setStateCommand(hasAlgaeCondition),
                        endEffectorSubsystem::hasCoral), // replace with actual condition checking for coral
                    setStateCommand(noGamePieceCondition),
                    () -> !endEffectorSubsystem.hasCoral()
                            || !endEffectorSubsystem.hasAlgae());
    }
}