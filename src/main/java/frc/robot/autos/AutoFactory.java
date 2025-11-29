package frc.robot.autos;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.config.CameraConfiguration.Location;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.superstructure.Superstructure;

import java.util.Set;

/**
 * A factory for creating autonomous programs for a given {@link Auto}
 */
class AutoFactory {
    private final DriverStation.Alliance alliance;

    private final RobotContainer robotContainer;

    private final double DISTANCE_TO_MOVE_ARM_UP = Units.inchesToMeters(48.0);

    /**
     * Create a new <code>AutoFactory</code>.
     *
     * @param robotContainer The {@link RobotContainer}
     */
    AutoFactory(final DriverStation.Alliance alliance, final RobotContainer robotContainer) {
        this.alliance = alliance;
        this.robotContainer = robotContainer;
    }

    /* Autonomous program factories
     *
     * Factory methods should be added here for each autonomous program.
     * The factory methods must:
     *   1. Be package-private (i.e. no access modifier)
     *   2. Accept no parameters
     *   3. Return a link Command
     */

    Pair<Pose2d, Command> createLIKLAuto() {
        var initialPose = Constants.PointsForPathfinding.LEFT_AUTO_START_POSE;
        return Pair.of(
                initialPose,
                Commands.sequence(
                        followThenScore(
                                Constants.ReefConstants.ReefFaces.IJ,
                                Superstructure.WantedSuperState.RIGHT_L4,
                                DISTANCE_TO_MOVE_ARM_UP,
                                10.0),
                        followThenIntakeFromStation(
                                FieldConstants.getLeftStationPickup(alliance), Units.feetToMeters(10.0)),
                        followThenScore(
                                Constants.ReefConstants.ReefFaces.KL,
                                Superstructure.WantedSuperState.LEFT_L4,
                                DISTANCE_TO_MOVE_ARM_UP,
                                Units.feetToMeters(10.0)),
                        followThenIntakeFromStation(
                                FieldConstants.getLeftStationPickup(alliance), Units.feetToMeters(10.0)),
                        followThenScoreWithNonDefaultMaxVelocity(
                                Constants.ReefConstants.ReefFaces.KL,
                                Superstructure.WantedSuperState.RIGHT_L4,
                                Units.feetToMeters(10.0)),
                        setState(Superstructure.WantedSuperState.HOME)));
    }

    Pair<Pose2d, Command> createRFDCAuto() {
        var initialPose = Constants.PointsForPathfinding.RIGHT_AUTO_START_POSE;
        return Pair.of(
                initialPose,
                Commands.sequence(
                        followThenScore(
                                Constants.ReefConstants.ReefFaces.EF,
                                Superstructure.WantedSuperState.RIGHT_L4,
                                DISTANCE_TO_MOVE_ARM_UP,
                                10.0),
                        followThenIntakeFromStation(
                                FieldConstants.getRightStationPickup(alliance), Units.feetToMeters(10.0)),
                        followThenScore(
                                Constants.ReefConstants.ReefFaces.CD,
                                Superstructure.WantedSuperState.RIGHT_L4,
                                DISTANCE_TO_MOVE_ARM_UP,
                                Units.feetToMeters(10.0)),
                        followThenIntakeFromStation(
                                FieldConstants.getRightStationPickup(alliance), Units.feetToMeters(10.0)),
                        followThenScoreWithNonDefaultMaxVelocity(
                                Constants.ReefConstants.ReefFaces.CD,
                                Superstructure.WantedSuperState.LEFT_L4,
                                Units.feetToMeters(10.0)),
                        setState(Superstructure.WantedSuperState.HOME)));
    }

    Pair<Pose2d, Command> createMG2NAuto() {

        var initialPose = Constants.PointsForPathfinding.MIDDLE_AUTO_START_POSE;

        return Pair.of(
                initialPose,
                Commands.sequence(
                        followThenScore(
                                Constants.ReefConstants.ReefFaces.GH,
                                Superstructure.WantedSuperState.RIGHT_L4,
                                DISTANCE_TO_MOVE_ARM_UP,
                                10.0)));
    }

    private String trajectoryName(final Location start, final Location end) {
        var name = "%S_TO_%S".formatted(start, end);
        var x = "%S_%S".formatted(alliance, name);
        System.out.println("%S_%S".formatted(alliance, name));
        return x;
    }

    Command setState(Superstructure.WantedSuperState state) {
        return robotContainer.getSuperstructure().setStateCommand(state);
    }

    // Command followTrajectory(Trajectory<SwerveSample> trajectory) {
        // return new InstantCommand(() -> robotContainer.getSwerveSubsystem().setDesiredChoreoTrajectory(trajectory));
    // }

    Command driveToPoint(Pose2d point, double maxVelocityOutputForDriveToPoint) {
        return new InstantCommand(() -> robotContainer
                        .getSwerveSubsystem()
                        .setDesiredPoseForDriveToPointWithConstraints(point, maxVelocityOutputForDriveToPoint, 1.0))
                .andThen(new WaitUntilCommand(
                        () -> robotContainer.getSwerveSubsystem().isAtDriveToPointSetpoint()));
    }

    Command driveToPointWithUnconstrainedMaxVelocity(Pose2d point, double maxVelocityOutputForDriveToPoint) {
        return new InstantCommand(() -> robotContainer
                        .getSwerveSubsystem()
                        .setDesiredPoseForDriveToPointWithConstraints(
                                point, maxVelocityOutputForDriveToPoint, Double.NaN))
                .andThen(new WaitUntilCommand(
                        () -> robotContainer.getSwerveSubsystem().isAtDriveToPointSetpoint()));
    }

    private Command followThenScore(
            Constants.ReefConstants.ReefFaces reefFaces,
            Superstructure.WantedSuperState scoreState,
            double distanceFromEndOfPathtoMoveArmUp,
            double maxVelocity) {
        var desiredPose = getAutoScoringPose(reefFaces, scoreState);
        return ((driveToPoint(desiredPose, maxVelocity)
                        .alongWith(new WaitUntilCommand(() ->
                                        robotContainer.getSwerveSubsystem().getDistanceFromDriveToPointSetpoint()
                                                < distanceFromEndOfPathtoMoveArmUp)
                                .andThen(setState(scoreState)))))
                .andThen(waitForCoralRelease().raceWith(new WaitCommand(1.0)));
    }

    private Command followThenScore(
            Constants.ReefConstants.ReefFaces reefFaces,
            Superstructure.WantedSuperState scoreState,
            double distanceFromEndOfPathtoMoveArmUp) {
        var desiredPose = getAutoScoringPose(reefFaces, scoreState);
        return ((driveToPoint(desiredPose, Units.feetToMeters(8.0))
                        .alongWith(new WaitUntilCommand(() ->
                                        robotContainer.getSwerveSubsystem().getDistanceFromDriveToPointSetpoint()
                                                < distanceFromEndOfPathtoMoveArmUp)
                                .andThen(setState(scoreState)))))
                .andThen(waitForCoralRelease().raceWith(new WaitCommand(1.0)));
    }

    private Command followThenScoreWithNonDefaultMaxVelocity(
            Constants.ReefConstants.ReefFaces reefFaces,
            Superstructure.WantedSuperState scoreState,
            double maxVelocity) {
        var desiredPose = getAutoScoringPose(reefFaces, scoreState);
        return ((driveToPoint(desiredPose, maxVelocity).alongWith(setState(scoreState))))
                .andThen(waitForCoralRelease().raceWith(new WaitCommand(1.0)));
    }

    private Command followThenScoreWithMinimumReleaseTime(
            Constants.ReefConstants.ReefFaces reefFaces, Superstructure.WantedSuperState scoreState) {
        var desiredPose = getAutoScoringPose(reefFaces, scoreState);
        return ((driveToPoint(desiredPose, Units.feetToMeters(12.0)).alongWith(setState(scoreState))))
                .andThen(new WaitCommand(0.5).andThen(waitForCoralRelease()).raceWith(new WaitCommand(1.0)));
    }

    private Command followThenScore(
            Constants.ReefConstants.ReefFaces reefFaces, Superstructure.WantedSuperState scoreState) {
        var desiredPose = getAutoScoringPose(reefFaces, scoreState);
        return ((driveToPoint(desiredPose, Units.feetToMeters(12.0)).alongWith(setState(scoreState))))
                .andThen(waitForCoralRelease().raceWith(new WaitCommand(1.0)));
    }

    private Command followThenScore(
            Constants.ReefConstants.ReefFaces reefFaces,
            Trajectory<SwerveSample> path,
            Superstructure.WantedSuperState scoreState) {
        // var noCoralState = (scoreState == Superstructure.WantedSuperState.LEFT_L2
                        // || scoreState == Superstructure.WantedSuperState.LEFT_L3
                        // || scoreState == Superstructure.WantedSuperState.LEFT_L4)
                // ? Superstructure.WantedSuperState.FORCE_RELOCALIZE_LEFT
                // : Superstructure.WantedSuperState.FORCE_RELOCALIZE_RIGHT;
        return followThenScore(reefFaces, path, scoreState
        //, noCoralState
         );
    }

    // private Command followThenScore(
    //         Constants.ReefConstants.ReefFaces reefFaces,
    //         Trajectory<SwerveSample> path,
    //         Superstructure.WantedSuperState scoreState,
    //         Superstructure.WantedSuperState noCoralState) {
    //     return (followTrajectory(path)
    //                     .andThen(new WaitUntilCommand(
    //                             robotContainer.getSwerveSubsystem()::isAtEndOfChoreoTrajectoryOrDriveToPoint)))
    //             .alongWith(new WaitUntilCommand(
    //                             () -> robotContainer.getSwerveSubsystem().getRobotDistanceFromChoreoEndpoint()
    //                                     < DISTANCE_TO_MOVE_ARM_UP)
    //                     .andThen(new ConditionalCommand(
    //                             followThenScore(reefFaces, scoreState),
    //                             setState(noCoralState),
    //                             () -> robotContainer.getSuperstructure().hasCoral())));
    // }

    public Pose2d getIntakePose(Translation2d intakeLocation) {
        var angle = intakeLocation
                .minus(RobotState.getInstance()
                        .getRobotPoseFromSwerveDriveOdometry()
                        .getTranslation())
                .getAngle();
        // var translation = intakeLocation.plus(new Translation2d(2.0, angle));
        return new Pose2d(intakeLocation, angle);
    }

    private Command followThenIntakeFromStation(Pose2d intakePose, double intakeVelocity) {
        return (driveToPoint(intakePose, intakeVelocity)
                        .alongWith(setState(Superstructure.WantedSuperState.HP_INAKE))
                        .andThen(Commands.waitSeconds(2.0)))
                .raceWith(waitForCoralPickup());
    }

    private Command followThenIntake(
            Pose2d intakePose, Superstructure.WantedSuperState intakeState, double intakeVelocity) {
        return (driveToPoint(intakePose, intakeVelocity).alongWith(setState(intakeState)))
                .raceWith(waitForCoralPickup());
    }

    // private Command followAndIntakeFromMark(double velocity, Translation2d markLocation) {
    //     return new DeferredCommand(
    //             () -> driveToPoint(getIntakePose(markLocation), velocity)
    //                     .alongWith(setState(Superstructure.WantedSuperState.MARK_PUNCH)
    //                             .andThen(new WaitUntilCommand(() -> robotContainer
    //                                                     .getSwerveSubsystem()
    //                                                     .getDistanceFromDriveToPointSetpoint()
    //                                             < 0.75)
    //                                     .andThen(setState(Superstructure.WantedSuperState.INTAKE_CORAL_FROM_GROUND))))
    //                     .raceWith(waitForCoralPickup()),
    //             Set.of());
    // }

    private Command waitForCoralRelease() {
        return new WaitUntilCommand(() -> !robotContainer.getSuperstructure().hasCoral());
    }

    private Command waitForCoralPickup() {
        return new WaitUntilCommand(() -> robotContainer.getSuperstructure().hasCoral());
    }

    public Pose2d getAutoScoringPose(
            Constants.ReefConstants.ReefFaces reefFaces, Superstructure.WantedSuperState superState) {
        var map = alliance == DriverStation.Alliance.Blue
                ? Constants.ReefConstants.blueAllianceReefFacesToIds
                : Constants.ReefConstants.redAllianceReefFacesToIds;
        var id = map.get(reefFaces);
        var scoringSide = (superState == Superstructure.WantedSuperState.LEFT_L2
                        || superState == Superstructure.WantedSuperState.LEFT_L3
                        || superState == Superstructure.WantedSuperState.LEFT_L4)
                ? Constants.SuperstructureConstants.ScoringSide.LEFT
                : Constants.SuperstructureConstants.ScoringSide.RIGHT;
        return FieldConstants.getDesiredFinalScoringPoseForCoral(
                id, scoringSide, Constants.SuperstructureConstants.ScoringDirection.BACK);
    }
}
