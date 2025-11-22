package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.config.CameraConfiguration;

public interface VisionIO {
    default void updateInputs(VisionIOInputs inputs) {}

    default void setLeds(boolean on) {}

    default CameraConfiguration.Location getLimelightLocation() {
        return CameraConfiguration.Location.NONE;
    }

    default String getLimelightName() {
        return "";
    }

    default void setThrottleValue(int throttleValue) {}

    @AutoLog
    class VisionIOInputs {
        public boolean hasTargets = false;
        public Rotation2d horizontalAngleToTarget = new Rotation2d();
        public int tagId = -1;
        public double targetHeight;
        public double distanceToTagMeters;
        public Rotation2d angleEncompassingTag;
        public Pose2d robotPoseBasedOffDistanceCalcAndTagLocation;
    }

    void updateInputs(VisionIOInputsAutoLogged inputs);
}
