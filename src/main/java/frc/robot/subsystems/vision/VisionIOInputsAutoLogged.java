package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.inputs.LoggableInputs;

import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;

public class VisionIOInputsAutoLogged extends VisionIOInputs implements LoggableInputs {
    @Override
    public void toLog(org.littletonrobotics.junction.LogTable table) {
        table.put("hasTargets", hasTargets);
        table.put("horizontalAngleToTarget", horizontalAngleToTarget);
        table.put("tagId", tagId);
        table.put("targetHeight", targetHeight);
        table.put("distanceToTagMeters", distanceToTagMeters);
        table.put("angleEncompassingTag", angleEncompassingTag);
        table.put("robotPoseBasedOffDistanceCalcAndTagLocation", robotPoseBasedOffDistanceCalcAndTagLocation);
    }

    @Override
    public void fromLog(org.littletonrobotics.junction.LogTable table) {
        hasTargets                                  = table.get("hasTargets", hasTargets);
        horizontalAngleToTarget                     = table.get("horizontalAngleToTarget", horizontalAngleToTarget);
        tagId                                       = table.get("tagId", tagId);
        targetHeight                                = table.get("targetHeight", targetHeight);
        distanceToTagMeters                         = table.get("distanceToTagMeters", distanceToTagMeters);
        angleEncompassingTag                        = table.get("angleEncompassingTag", angleEncompassingTag);
        robotPoseBasedOffDistanceCalcAndTagLocation = table.get("robotPoseBasedOffDistanceCalcAndTagLocation", robotPoseBasedOffDistanceCalcAndTagLocation);
    }

}
