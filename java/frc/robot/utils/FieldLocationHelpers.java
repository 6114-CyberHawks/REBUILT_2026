package frc.robot.utils;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import frc.robot.LimelightHelpers;

public final class FieldLocationHelpers {

    /*
     * =======================
     * APRILTAG FIELD LAYOUT
     * =======================
     */
    private static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFields.k2026RebuiltAndymark
            .loadAprilTagLayoutField(); // Rebuilt field

    /*
     * =======================
     * ROBOT POSE (MEGATAG2)
     * =======================
     */
    public static Pose2d getRobotFieldPoseMegaTag(
            String limelightName,
            boolean isRedAlliance) {
        LimelightHelpers.PoseEstimate estimate = isRedAlliance
                ? LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(limelightName)
                : LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

        if (estimate == null || estimate.tagCount == 0) {
            return null;
        }

        return estimate.pose;
    }

    /*
     * =======================
     * TAG POSE FROM LAYOUT
     * =======================
     */
    public static Pose3d getTagPose(int tagID) {
        return FIELD_LAYOUT.getTagPose(tagID).orElse(null);
    }

    /*
     * =======================
     * DISTANCE TO TAG (METERS)
     * =======================
     */
    public static double getDistanceToTagMeters(
            String limelightName,
            int tagID,
            boolean isRedAlliance) {
        Pose2d robotPoseOpt = getRobotFieldPoseMegaTag(limelightName, isRedAlliance);
        Pose3d tagPoseOpt = getTagPose(tagID);

        if (robotPoseOpt == null || tagPoseOpt == null) {
            return -1;
        }

        Translation2d robot = robotPoseOpt.getTranslation();
        Translation2d tag = tagPoseOpt.toPose2d().getTranslation();

        return robot.getDistance(tag);
    }

    /*
     * =======================
     * FIELD-RELATIVE ANGLE
     * =======================
     */
    public static Rotation2d getFieldAngleToTag(
            String limelightName,
            int tagID,
            boolean isRedAlliance) {
        Pose2d robotPoseOpt = getRobotFieldPoseMegaTag(limelightName, isRedAlliance);
        Pose3d tagPoseOpt = getTagPose(tagID);

        if (robotPoseOpt == null || tagPoseOpt == null) {
            return null;
        }

        Translation2d robot = robotPoseOpt.getTranslation();
        Translation2d tag = tagPoseOpt.toPose2d().getTranslation();

        Translation2d delta = tag.minus(robot);
        return new Rotation2d(delta.getX(), delta.getY());
    }

    /*
     * =======================
     * ROBOT-RELATIVE ANGLE
     * (TURN-TO-TARGET)
     * =======================
     */
    public static Rotation2d getRobotRelativeAngleToTag(
            String limelightName,
            int tagID,
            boolean isRedAlliance) {
        Rotation2d fieldAngleOpt = getFieldAngleToTag(limelightName, tagID, isRedAlliance);
        Pose2d robotPoseOpt = getRobotFieldPoseMegaTag(limelightName, isRedAlliance);

        if (fieldAngleOpt == null || robotPoseOpt == null) {
            return null;
        }

        return fieldAngleOpt.minus(robotPoseOpt.getRotation());
    }
}
