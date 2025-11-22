package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.ArmPosition;

public final class ArmPoseConstants {
                                                                // Shoulder        // Wrist
    public static final ArmPosition ZEROED = new ArmPosition(Rotation2d.kZero, Rotation2d.kZero);
    public static final ArmPosition DRIVE =        // Shoulder                          // Wrist
            new ArmPosition(Rotation2d.fromDegrees(82), Rotation2d.fromDegrees(85));
    public static final ArmPosition HP_INTAKE_START =
            new ArmPosition(Rotation2d.fromDegrees(45), Rotation2d.fromDegrees(180));
    public static final ArmPosition HP_INTAKE_HOLD =
            new ArmPosition(Rotation2d.fromDegrees(55), Rotation2d.fromDegrees(180));
    public static final ArmPosition GROUND_ALGAE =
            new ArmPosition(Rotation2d.fromDegrees(60), Rotation2d.fromDegrees(85));
    public static final ArmPosition REEF_ALGAE =
            new ArmPosition(Rotation2d.fromDegrees(40), Rotation2d.fromDegrees(-23));
    public static final ArmPosition NET =
            new ArmPosition(Rotation2d.fromDegrees(46), Rotation2d.fromDegrees(70));
    public static final ArmPosition PROCESSOR =
            new ArmPosition(Rotation2d.fromDegrees(72), Rotation2d.fromDegrees(125));
    public static final ArmPosition L1 =
            new ArmPosition(Rotation2d.fromDegrees(78), Rotation2d.fromDegrees(33));
    public static final ArmPosition L2 =
            new ArmPosition(Rotation2d.fromDegrees(50), Rotation2d.fromDegrees(131));
    public static final ArmPosition L3 =
            new ArmPosition(Rotation2d.fromDegrees(60), Rotation2d.fromDegrees(125));
    public static final ArmPosition L4 =
            new ArmPosition(Rotation2d.fromDegrees(60), Rotation2d.fromDegrees(125));
    public static final ArmPosition CLIMB =
            new ArmPosition(Rotation2d.fromDegrees(89), Rotation2d.fromDegrees(85));
}
