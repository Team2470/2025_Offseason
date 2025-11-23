package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.Map;

public final class Constants {
    public static final class SuperstructureConstants {
        public static final double X_OFFSET_FROM_TAG_FOR_L1_BASE_SCORING_INCHES = 21.0;
        public static final double X_OFFSET_FROM_TAG_FOR_L1_TOP_SCORING_INCHES = 19.25;

        public static final double X_OFFSET_FROM_TAG_FOR_SCORING_INCHES = 22.0;
        public static final double X_OFFSET_FROM_TAG_FOR_INTAKING_ALGAE_INCHES = 18.0;
        public static final double X_OFFSET_FROM_TAG_FOR_INTERMEDIATE_INTAKING_ALGAE_INCHES = 30.0;
        public static final double X_OFFSET_FROM_TAG_FOR_BACKOUT_INTAKING_ALGAE_INCHES = 50.0;
        public static final double X_OFFSET_FROM_TAG_FOR_L1_BACKOUT_INCHES = 10.0;

        public static final double Y_OFFSET_FROM_TAG_FOR_SCORING_ON_REEF_INCHES = 6.5;
        public static final double Y_OFFSET_FROM_TAG_FOR_SCORING_L1_INCHES = 9.0;

        public enum ScoringDirection {
            FRONT,
            BACK
        }

        public enum ScoringSide {
            RIGHT,
            LEFT
        }

        public enum AutomationLevel {
            AUTO_RELEASE,
            AUTO_DRIVE_AND_MANUAL_RELEASE,
            NO_AUTO_DRIVE
        }

        public enum ReefSelectionMethod {
            POSE,
            ROTATION
        }
    }

    public static final class ReefConstants {
        public enum ReefFaces {
            AB,
            CD,
            EF,
            GH,
            IJ,
            KL
        }

        public enum AlgaeIntakeLocation {
            L2,
            L3
        }

        public static final class AlgaeIntakeMapping {
            public final AlgaeIntakeLocation ID;

            public AlgaeIntakeMapping(AlgaeIntakeLocation id) {
                ID = id;
            }
        }

        public static final class ScoringCoralMappingRotationToTagID {
            public final int ID;

            public ScoringCoralMappingRotationToTagID(int id) {
                ID = id;
            }
        }

        public static final Map<Pose2d, Integer> blueAlliancePoseToTagIDsMap = Map.of(
                FieldConstants.getTagPose(21).toPose2d(), 21,
                FieldConstants.getTagPose(20).toPose2d(), 20,
                FieldConstants.getTagPose(19).toPose2d(), 19,
                FieldConstants.getTagPose(18).toPose2d(), 18,
                FieldConstants.getTagPose(17).toPose2d(), 17,
                FieldConstants.getTagPose(22).toPose2d(), 22);

        public static final Map<Pose2d, Integer> redAlliancePoseToTagIDsMap = Map.of(
                FieldConstants.getTagPose(6).toPose2d(), 6,
                FieldConstants.getTagPose(7).toPose2d(), 7,
                FieldConstants.getTagPose(8).toPose2d(), 8,
                FieldConstants.getTagPose(9).toPose2d(), 9,
                FieldConstants.getTagPose(10).toPose2d(), 10,
                FieldConstants.getTagPose(11).toPose2d(), 11);

        public static final Map<Rotation2d, ScoringCoralMappingRotationToTagID> redAllianceAngleToTagIDsMap = Map.of(
                Rotation2d.fromDegrees(-60),
                new ScoringCoralMappingRotationToTagID(9),
                Rotation2d.fromDegrees(-120),
                new ScoringCoralMappingRotationToTagID(8),
                Rotation2d.k180deg,
                new ScoringCoralMappingRotationToTagID(7),
                Rotation2d.fromDegrees(120),
                new ScoringCoralMappingRotationToTagID(6),
                Rotation2d.fromDegrees(60),
                new ScoringCoralMappingRotationToTagID(11),
                Rotation2d.kZero,
                new ScoringCoralMappingRotationToTagID(10));

        public static final Map<Rotation2d, ScoringCoralMappingRotationToTagID> blueAllianceAngleToTagIDsMap = Map.of(
                Rotation2d.fromDegrees(-60),
                new ScoringCoralMappingRotationToTagID(19),
                Rotation2d.fromDegrees(-120),
                new ScoringCoralMappingRotationToTagID(20),
                Rotation2d.k180deg,
                new ScoringCoralMappingRotationToTagID(21),
                Rotation2d.fromDegrees(120),
                new ScoringCoralMappingRotationToTagID(22),
                Rotation2d.fromDegrees(60),
                new ScoringCoralMappingRotationToTagID(17),
                Rotation2d.kZero,
                new ScoringCoralMappingRotationToTagID(18));

        public static final Map<Rotation2d, AlgaeIntakeMapping> redAllianceAlgae = Map.of(
                Rotation2d.fromDegrees(0),
                new AlgaeIntakeMapping(AlgaeIntakeLocation.L2),
                Rotation2d.fromDegrees(60),
                new AlgaeIntakeMapping(AlgaeIntakeLocation.L3),
                Rotation2d.fromDegrees(120),
                new AlgaeIntakeMapping(AlgaeIntakeLocation.L2),
                Rotation2d.fromDegrees(180),
                new AlgaeIntakeMapping(AlgaeIntakeLocation.L3),
                Rotation2d.fromDegrees(-120),
                new AlgaeIntakeMapping(AlgaeIntakeLocation.L2),
                Rotation2d.fromDegrees(-60),
                new AlgaeIntakeMapping(AlgaeIntakeLocation.L3));

        public static final Map<Rotation2d, AlgaeIntakeMapping> blueAllianceAlgae = Map.of(
                Rotation2d.fromDegrees(0),
                new AlgaeIntakeMapping(AlgaeIntakeLocation.L3),
                Rotation2d.fromDegrees(60),
                new AlgaeIntakeMapping(AlgaeIntakeLocation.L2),
                Rotation2d.fromDegrees(120),
                new AlgaeIntakeMapping(AlgaeIntakeLocation.L3),
                Rotation2d.fromDegrees(180),
                new AlgaeIntakeMapping(AlgaeIntakeLocation.L2),
                Rotation2d.fromDegrees(-120),
                new AlgaeIntakeMapping(AlgaeIntakeLocation.L3),
                Rotation2d.fromDegrees(-60),
                new AlgaeIntakeMapping(AlgaeIntakeLocation.L2));

        public static final Map<ReefFaces, Integer> redAllianceReefFacesToIds = Map.of(
                ReefFaces.AB, 7,
                ReefFaces.CD, 8,
                ReefFaces.EF, 9,
                ReefFaces.GH, 10,
                ReefFaces.IJ, 11,
                ReefFaces.KL, 6);

        public static final Map<ReefFaces, Integer> blueAllianceReefFacesToIds = Map.of(
                ReefFaces.AB, 18,
                ReefFaces.CD, 17,
                ReefFaces.EF, 22,
                ReefFaces.GH, 21,
                ReefFaces.IJ, 20,
                ReefFaces.KL, 19);
    }

    public static final class ArmConstants {
        public static final double ZERO_VELOCITY_TIME_PERIOD = 0.5;

        // Distance from pivot point to arm base
        public static final double ARM_PIVOT_OFFSET =
                Units.Meter.convertFrom(6.75 - 0.125, Units.Inch); // TODO: UPDATE THIS
        // Distance from origin of robot to pivot point
        public static final Translation2d ORIGIN_PIVOT_OFFSET = new Translation2d(
                -Units.Meter.convertFrom(10.25, Units.Inch),
                Units.Meter.convertFrom(13.25, Units.Inch)); // TODO: UPDATE THIS

        public static final double SHOULDER_ZERO_VELOCITY_THRESHOLD_RADIANS_PER_SECOND = 0.08;
        public static final double WRIST_ZERO_VELOCITY_THRESHOLD_RADIANS_PER_SECOND = 0.03;

        public static final double SHOULDER_SETPOINT_TOLERANCE_DEGREES = 1.0;
        public static final double WRIST_SETPOINT_TOLERANCE_DEGREES = 2.0;
        public static final double TOLERANCE_FOR_EXTENSION_DEGREES = 40.0;

        public static final double SHOULDER_EXTENSION_COUPLING_RATIO = (32.0 / 16.0) * (40.0 / 26.0) * (50.0 / 20.0);

        public static final double SHOULDER_GEAR_RATIO = (60.0 / 12.0) * (60.0 / 16.0) * (58.0 / 9.0);
        public static final double SHOULDER_POSITION_COEFFICIENT = 2 * Math.PI / SHOULDER_GEAR_RATIO;

        public static final double SHOULDER_ACCELERATION = Units.Radians.convertFrom(600, Units.Degree);
        public static final double SHOULDER_VELOCITY = Units.Radian.convertFrom(1000, Units.Degree);

        public static final double WRIST_ACCELERATION = Units.Radian.convertFrom(4500, Units.Degree);
        public static final double WRIST_VELOCITY = Units.Radian.convertFrom(2000, Units.Degree);

        public static final double EXTENSION_ACCELERATION = Units.Meter.convertFrom(400, Units.Inch);
        public static final double EXTENSION_VELOCITY = Units.Meter.convertFrom(200, Units.Inch);

        public static final double WRIST_ACCELERATION_CONSTRAINT =
                WRIST_ACCELERATION / ArmConstants.WRIST_POSITION_COEFFICIENT;
        public static final double WRIST_VELOCITY_CONSTRAINT = WRIST_VELOCITY / ArmConstants.WRIST_POSITION_COEFFICIENT;

        public static final double SHOULDER_ACCELERATION_CONSTRAINT =
                SHOULDER_ACCELERATION / SHOULDER_POSITION_COEFFICIENT;
        public static final double SHOULDER_VELOCITY_CONSTRAINT = SHOULDER_VELOCITY / SHOULDER_POSITION_COEFFICIENT;

        public static final double WRIST_GEAR_RATIO = (50.0 / 9.0) * (38.0 / 12.0) * (38.0 / 12.0);
        public static final double WRIST_POSITION_COEFFICIENT = 2 * Math.PI / WRIST_GEAR_RATIO;

        public static final double SHOULDER_ZEROING_DUTY_CYCLE = -0.05;
        public static final double WRIST_ZEROING_DUTY_CYCLE = 0.07;

        public static final double SHOULDER_DRIVEN_HOME_ANGLE_DEGREES = -1.647;
        public static final double WRIST_DRIVEN_HOME_ANGLE_DEGREES = 136.0;

        public static final double SHOULDER_BUTTON_HOME_ANGLE_DEGREES = -0.5;
        public static final double WRIST_BUTTON_HOME_ANGLE_DEGREES = 134.74;
    }

    public static final class ClimberConstants {
        public static final double INTAKE_MOTOR_VOLTAGE = 12.0;
        public static final double REJECT_MOTOR_VOLTAGE = -12.0;

        public static final double CLIMBER_PULLEY_DIAMETER = Units.Meter.convertFrom(0.25 * 12 / Math.PI, Units.Inch);
        public static final double CLIMBER_GEAR_RATIO = (66.0 / 9.0) * (50.0 / 14.0);
        public static final double CLIMBER_ACCELERATION = Units.Meter.convertFrom(40.0, Units.Inch);
        public static final double CLIMBER_VELOCITY = Units.Meter.convertFrom(10.0, Units.Inch);
        public static final double CLIMBER_CARRIAGE_INTAKE_POSITION = Units.Meter.convertFrom(7.5, Units.Inch);
        public static final double CLIMBER_CARRIAGE_SETPOINT_TOLERANCE = Units.Meter.convertFrom(0.1, Units.Inch);
        public static final double CLIMBER_CARRIAGE_CAGE_SUPPLY_CURRENT_THRESHOLD = 15.0;
        public static final double CLIMBER_CARRIAGE_CAGE_VELOCITY_RPS_THRESHOLD = 70.0;
        public static final double CLIMBER_DRIVEN_HOME_RESET_POSITION_METERS =
                Units.Meter.convertFrom(-0.04, Units.Inch);

        public static final double CLIMBER_BUTTON_HOME_RESET_POSITION_METERS = 0.0;

        public static final double CLIMBER_ZERO_VELOCITY_THRESHOLD_METERS_PER_SECOND =
                Units.Meter.convertFrom(0.1, Units.Inch);
        public static final double CLIMBER_ZERO_VELOCITY_TIME_PERIOD = 0.15;
        public static final double CLIMBER_DUTY_CYCLE_FOR_ZEROING = -0.07;

        public static final double CLIMBER_POSITION_COEFFICIENT =
                Math.PI * CLIMBER_PULLEY_DIAMETER / CLIMBER_GEAR_RATIO;

        public static final double CLIMBER_ACCELERATION_CONSTRAINT =
                CLIMBER_ACCELERATION / ClimberConstants.CLIMBER_POSITION_COEFFICIENT;
        public static final double CLIMBER_VELOCITY_CONSTRAINT =
                CLIMBER_VELOCITY / ClimberConstants.CLIMBER_POSITION_COEFFICIENT;
    }

    public static final class IntakeConstants {
        public static final class OffVoltages {
            public static final double OFF_ALGAE_VOLTAGE = 0.0;
            public static final double OFF_CORAL_VOLTAGE = 12.0;
        }

        public static final class CollectingVoltages {
            public static final double COLLECTING_ALGAE_VOLTAGE = -12.0;
            public static final double COLLECTING_CORAL_VOLTAGE = 12.0;
        }

        public static final class EjectingVoltages {
            public static final double EJECTING_ALGAE_VOLTAGE = 12.0;
            public static final double EJECTING_PROCESSOR_TOP_ALGAE_VOLTAGE = 1.0;
            public static final double EJECTING_CORAL_VOLTAGE = -12.0;
            public static final double EJECTING_CORAL_VOLTAGE_L1 = -1.0;
        }

        public static final class HoldingVoltages {
            public static final double HOLDING_ALGAE_VOLTAGE = -12.0;
            public static final double HOLDING_ALGAE_HARDER_VOLTAGE = -12.0;
            public static final double HOLDING_CORAL_VOLTAGE = 0.0;
        }

        public static final double TOP_ROLLER_CURRENT_THRESHOLD_FOR_ALGAE_DETECTION = 9.0;
        public static final double TOP_ROLLER_VELOCITY_RPS_THRESHOLD_FOR_ALGAE_DETECTION_ALLOWANCE = -80.0;
        public static final double TOP_ROLLER_VELOCITY_RPS_THRESHOLD_FOR_ALGAE_DETECTION_WHILE_INTAKING = -20.0;
        public static final double TOP_ROLLER_VELOCITY_RPS_THRESHOLD_FOR_ALGAE_DETECTION_WHILE_HOLDING = -70.0;
    }

    public static final boolean SILENCE_JOYSTICK_WARNINGS_IN_SIMULATOR = true;
    public static final Mode currentMode = Mode.REAL;
    public static final String operatorDashboardName = "Dashboard";
    public static final String autoChooserName = "SmartDashboard/Auto/Programs";

    public static final class OperatorConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
    }

    public static final class PointsForPathfinding {

        public static boolean isBlueAlliance() {
            return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue;
        }

        public static final Translation2d LEFT_HP_INTAKE_POSITION_BLUE = new Translation2d(
            Units.Meter.convertFrom(1.250, Units.Meter),
            Units.Meter.convertFrom(7.000, Units.Meter)
        );
        public static final Translation2d RIGHT_HP_INTAKE_POSITION_BLUE = new Translation2d(
            Units.Meter.convertFrom(1.250, Units.Meter),
            Units.Meter.convertFrom(1.000, Units.Meter)
        );
        public static final Translation2d A_POST_POSITION_BLUE = new Translation2d(
            Units.Meter.convertFrom(3.150, Units.Meter),
            Units.Meter.convertFrom(4.175, Units.Meter)
        );
        public static final Translation2d B_POST_POSITION_BLUE = new Translation2d(
            Units.Meter.convertFrom(3.150, Units.Meter),
            Units.Meter.convertFrom(3.850, Units.Meter)
        );
        public static final Translation2d C_POST_POSITION_BLUE = new Translation2d(
            Units.Meter.convertFrom(3.650, Units.Meter),
            Units.Meter.convertFrom(2.950, Units.Meter)
        );
        public static final Translation2d D_POST_POSITION_BLUE = new Translation2d(
            Units.Meter.convertFrom(3.975, Units.Meter),
            Units.Meter.convertFrom(2.800, Units.Meter)
        );
        public static final Translation2d E_POST_POSITION_BLUE = new Translation2d(
            Units.Meter.convertFrom(5.000, Units.Meter),
            Units.Meter.convertFrom(2.800, Units.Meter)
        );
        public static final Translation2d F_POST_POSITION_BLUE = new Translation2d(
            Units.Meter.convertFrom(5.300, Units.Meter),
            Units.Meter.convertFrom(2.950, Units.Meter)
        );
        public static final Translation2d G_POST_POSITION_BLUE = new Translation2d(
            Units.Meter.convertFrom(5.850, Units.Meter),
            Units.Meter.convertFrom(3.850, Units.Meter)
        );
        public static final Translation2d H_POST_POSITION_BLUE = new Translation2d(
            Units.Meter.convertFrom(5.850, Units.Meter),
            Units.Meter.convertFrom(4.175, Units.Meter)
        );
        public static final Translation2d I_POST_POSITION_BLUE = new Translation2d(
            Units.Meter.convertFrom(5.300, Units.Meter),
            Units.Meter.convertFrom(5.100, Units.Meter)
        );
        public static final Translation2d J_POST_POSITION_BLUE = new Translation2d(
            Units.Meter.convertFrom(5.000, Units.Meter),
            Units.Meter.convertFrom(5.250, Units.Meter)
        );
        public static final Translation2d K_POST_POSITION_BLUE = new Translation2d(
            Units.Meter.convertFrom(3.975, Units.Meter),
            Units.Meter.convertFrom(5.250, Units.Meter)
        );
        public static final Translation2d L_POST_POSITION_BLUE = new Translation2d(
            Units.Meter.convertFrom(3.650, Units.Meter),
            Units.Meter.convertFrom(5.100, Units.Meter)
        );

        public static final Translation2d LEFT_HP_INTAKE_POSITION_RED = new Translation2d(
            Units.Meter.convertFrom(16.250, Units.Meter),
            Units.Meter.convertFrom(1.000, Units.Meter)
        );
        public static final Translation2d RIGHT_HP_INTAKE_POSITION_RED = new Translation2d(
            Units.Meter.convertFrom(16.250, Units.Meter),
            Units.Meter.convertFrom(7.000, Units.Meter)
        );
        public static final Translation2d A_POST_POSITION_RED = new Translation2d(
            Units.Meter.convertFrom(14.400, Units.Meter),
            Units.Meter.convertFrom(3.850, Units.Meter)
        );
        public static final Translation2d B_POST_POSITION_RED = new Translation2d(
            Units.Meter.convertFrom(14.400, Units.Meter),
            Units.Meter.convertFrom(4.175, Units.Meter)
        );
        public static final Translation2d C_POST_POSITION_RED = new Translation2d(
            Units.Meter.convertFrom(13.900, Units.Meter),
            Units.Meter.convertFrom(5.100, Units.Meter)
        );
        public static final Translation2d D_POST_POSITION_RED = new Translation2d(
            Units.Meter.convertFrom(13.600, Units.Meter),
            Units.Meter.convertFrom(5.250, Units.Meter)
        );
        public static final Translation2d E_POST_POSITION_RED = new Translation2d(
            Units.Meter.convertFrom(12.550, Units.Meter),
            Units.Meter.convertFrom(5.250, Units.Meter)
        );
        public static final Translation2d F_POST_POSITION_RED = new Translation2d(
            Units.Meter.convertFrom(12.250, Units.Meter),
            Units.Meter.convertFrom(5.250, Units.Meter)
        );
        public static final Translation2d G_POST_POSITION_RED = new Translation2d(
            Units.Meter.convertFrom(11.700, Units.Meter),
            Units.Meter.convertFrom(4.175, Units.Meter)
        );
        public static final Translation2d H_POST_POSITION_RED = new Translation2d(
            Units.Meter.convertFrom(11.700, Units.Meter),
            Units.Meter.convertFrom(3.850, Units.Meter)
        );
        public static final Translation2d I_POST_POSITION_RED = new Translation2d(
            Units.Meter.convertFrom(12.250, Units.Meter),
            Units.Meter.convertFrom(2.950, Units.Meter)
        );
        public static final Translation2d J_POST_POSITION_RED = new Translation2d(
            Units.Meter.convertFrom(12.550, Units.Meter),
            Units.Meter.convertFrom(2.800, Units.Meter)
        );
        public static final Translation2d K_POST_POSITION_RED = new Translation2d(
            Units.Meter.convertFrom(13.600, Units.Meter),
            Units.Meter.convertFrom(2.800, Units.Meter)
        );
        public static final Translation2d L_POST_POSITION_RED = new Translation2d(
            Units.Meter.convertFrom(13.900, Units.Meter),
            Units.Meter.convertFrom(2.950, Units.Meter)
        );


        public static final Translation2d LEFT_HP_INTAKE_POSITION   = isBlueAlliance() ? LEFT_HP_INTAKE_POSITION_BLUE           : LEFT_HP_INTAKE_POSITION_RED;
        public static final Translation2d RIGHT_HP_INTAKE_POSITION  = isBlueAlliance() ? RIGHT_HP_INTAKE_POSITION_BLUE          : RIGHT_HP_INTAKE_POSITION_RED;
        public static final Translation2d A_POST_POSITION           = isBlueAlliance() ? A_POST_POSITION_BLUE                   : A_POST_POSITION_RED;
        public static final Translation2d B_POST_POSITION           = isBlueAlliance() ? B_POST_POSITION_BLUE                   : B_POST_POSITION_RED;
        public static final Translation2d C_POST_POSITION           = isBlueAlliance() ? C_POST_POSITION_BLUE                   : C_POST_POSITION_RED;
        public static final Translation2d D_POST_POSITION           = isBlueAlliance() ? D_POST_POSITION_BLUE                   : D_POST_POSITION_RED;
        public static final Translation2d E_POST_POSITION           = isBlueAlliance() ? E_POST_POSITION_BLUE                   : E_POST_POSITION_RED;
        public static final Translation2d F_POST_POSITION           = isBlueAlliance() ? F_POST_POSITION_BLUE                   : F_POST_POSITION_RED;
        public static final Translation2d G_POST_POSITION           = isBlueAlliance() ? G_POST_POSITION_BLUE                   : G_POST_POSITION_RED;
        public static final Translation2d H_POST_POSITION           = isBlueAlliance() ? H_POST_POSITION_BLUE                   : H_POST_POSITION_RED;
        public static final Translation2d I_POST_POSITION           = isBlueAlliance() ? I_POST_POSITION_BLUE                   : I_POST_POSITION_RED;
        public static final Translation2d J_POST_POSITION           = isBlueAlliance() ? J_POST_POSITION_BLUE                   : J_POST_POSITION_RED;
        public static final Translation2d K_POST_POSITION           = isBlueAlliance() ? K_POST_POSITION_BLUE                   : K_POST_POSITION_RED;
        public static final Translation2d L_POST_POSITION           = isBlueAlliance() ? L_POST_POSITION_BLUE                   : L_POST_POSITION_RED;
        public static final Rotation2d    LEFT_HP_INTAKE_ANGLE      = isBlueAlliance() ? Rotation2d.fromDegrees(-54)            : Rotation2d.fromDegrees(126);
        public static final Rotation2d    RIGHT_HP_INTAKE_ANGLE     = isBlueAlliance() ? Rotation2d.fromDegrees(54)     : Rotation2d.fromDegrees(-126);
        public static final Rotation2d    AB_FACE_ANGLE             = isBlueAlliance() ? Rotation2d.fromDegrees(0)      : Rotation2d.fromDegrees(180);
        public static final Rotation2d    CD_FACE_ANGLE             = isBlueAlliance() ? Rotation2d.fromDegrees(60)     : Rotation2d.fromDegrees(-120);
        public static final Rotation2d    EF_FACE_ANGLE             = isBlueAlliance() ? Rotation2d.fromDegrees(120)    : Rotation2d.fromDegrees(-60);
        public static final Rotation2d    GH_FACE_ANGLE             = isBlueAlliance() ? Rotation2d.fromDegrees(180)    : Rotation2d.fromDegrees(0);
        public static final Rotation2d    IJ_FACE_ANGLE             = isBlueAlliance() ? Rotation2d.fromDegrees(-120)           : Rotation2d.fromDegrees(60);
        public static final Rotation2d    KL_FACE_ANGLE             = isBlueAlliance() ? Rotation2d.fromDegrees(-60)            : Rotation2d.fromDegrees(120);

        public static final Pose2d LEFT_HP_INTAKE_POSE  = new Pose2d(LEFT_HP_INTAKE_POSITION, LEFT_HP_INTAKE_ANGLE);
        public static final Pose2d RIGHT_HP_INTAKE_POSE = new Pose2d(RIGHT_HP_INTAKE_POSITION, RIGHT_HP_INTAKE_ANGLE);
        public static final Pose2d A_POST_POSE          = new Pose2d(A_POST_POSITION, AB_FACE_ANGLE);
        public static final Pose2d B_POST_POSE          = new Pose2d(B_POST_POSITION, AB_FACE_ANGLE);
        public static final Pose2d C_POST_POSE          = new Pose2d(C_POST_POSITION, CD_FACE_ANGLE);
        public static final Pose2d D_POST_POSE          = new Pose2d(D_POST_POSITION, CD_FACE_ANGLE);
        public static final Pose2d E_POST_POSE          = new Pose2d(E_POST_POSITION, EF_FACE_ANGLE);
        public static final Pose2d F_POST_POSE          = new Pose2d(F_POST_POSITION, EF_FACE_ANGLE);
        public static final Pose2d G_POST_POSE          = new Pose2d(G_POST_POSITION, GH_FACE_ANGLE);
        public static final Pose2d H_POST_POSE          = new Pose2d(H_POST_POSITION, GH_FACE_ANGLE);
        public static final Pose2d I_POST_POSE          = new Pose2d(I_POST_POSITION, IJ_FACE_ANGLE);
        public static final Pose2d J_POST_POSE          = new Pose2d(J_POST_POSITION, IJ_FACE_ANGLE);
        public static final Pose2d K_POST_POSE          = new Pose2d(K_POST_POSITION, KL_FACE_ANGLE);
        public static final Pose2d L_POST_POSE          = new Pose2d(L_POST_POSITION, KL_FACE_ANGLE);
    }

    public static final class SysIdConstants {
        public static final Velocity<VoltageUnit> TRANSLATION_RAMP_RATE = null;
        public static final Voltage TRANSLATION_STEP_RATE = Units.Volts.of(7);
        public static final Time TRANSLATION_TIMEOUT = Units.Seconds.of(5);

        /* This is in radians per second², but SysId only supports "volts per second" */
        public static final Velocity<VoltageUnit> ROTATION_RAMP_RATE =
                Units.Volts.of(Math.PI / 6).per(Units.Second);
        /* This is in radians per second, but SysId only supports "volts" */
        public static final Voltage ROTATION_STEP_RATE = Units.Volts.of(Math.PI);
        public static final Time ROTATION_TIMEOUT = Units.Seconds.of(5);

        public static final Velocity<VoltageUnit> STEER_RAMP_RATE = null;
        public static final Voltage STEER_STEP_RATE = Units.Volts.of(7);
        public static final Time STEER_TIMEOUT = null;
    }

    public static enum Mode {
        /**
         * Running on a real robot.
         */
        REAL,

        /**
         * Running a physics simulator.
         */
        SIM,

        /**
         * Replaying from a log file.
         */
        REPLAY
    }
}
