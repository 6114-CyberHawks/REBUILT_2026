package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Autonomous LimeLight climb alignment using MegaTag1.
 * 
 * Designed for autonomous routines - uses existing LimelightHelpers v1.11.
 * Detects climb tags: Red (9, 10) or Blue (25, 26).
 * 
 * Two-tag detection provides high accuracy without MegaTag2.
 */
public class AutoLimelightClimbAlign extends Command {

    private final DriveSubsystem driveSubsystem;
    private static final String LL_NAME = "";

    // 2026 Climb AprilTag IDs
    private static final int[] RED_CLIMB_TAGS = {9, 10};
    private static final int[] BLUE_CLIMB_TAGS = {25, 26};

    // Alignment parameters
    private final double targetDistance;
    private final double positionTolerance;
    private final double rotationTolerance;

    // PID Controllers
    private final PIDController xPID;
    private final PIDController yPID;
    private final PIDController rotPID;

    // State
    private Pose2d targetPose = null;
    private boolean aligned = false;
    private int stableFrameCount = 0;
    private static final int REQUIRED_STABLE_FRAMES = 10;

    // Filtering
    private static final double SINGLE_TAG_MAX_AMBIGUITY = 0.7;
    private static final double SINGLE_TAG_MAX_DIST = 3.0;
    private static final double MULTI_TAG_MAX_DIST = 5.0;

    /**
     * Creates auto climb alignment with default parameters.
     * Target distance: 0.5m, Position tolerance: 5cm, Rotation tolerance: 2 degrees
     */
    public AutoLimelightClimbAlign(DriveSubsystem s_DriveSubsystem) {
        this(s_DriveSubsystem, 0.5, 0.05, 2.0);
    }

    /**
     * Creates auto climb alignment with custom parameters.
     */
    public AutoLimelightClimbAlign(DriveSubsystem s_driveSubsystem, 
                                    double targetDistanceMeters,
                                    double positionToleranceMeters,
                                    double rotationToleranceDegrees) {
        driveSubsystem = s_driveSubsystem;
        this.targetDistance = targetDistanceMeters;
        this.positionTolerance = positionToleranceMeters;
        this.rotationTolerance = rotationToleranceDegrees;

        xPID = new PIDController(2.0, 0.05, 0.1);
        yPID = new PIDController(2.0, 0.05, 0.1);
        rotPID = new PIDController(3.0, 0.05, 0.15);

        rotPID.enableContinuousInput(-180, 180);

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        targetPose = null;
        aligned = false;
        stableFrameCount = 0;

        xPID.reset();
        yPID.reset();
        rotPID.reset();

        LimelightHelpers.setPipelineIndex(LL_NAME, 0);

        System.out.println("[AUTO CLIMB] Starting LimeLight alignment (MegaTag1)");
        System.out.println("[AUTO CLIMB] Target distance: " + targetDistance + "m");
    }

    @Override
    public void execute() {
        PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(LL_NAME);

        if (!isValidEstimate(mt1)) {
            driveSubsystem.drive(0, 0, 0, true);
            stableFrameCount = 0;
            return;
        }

        int climbTagCount = countClimbTags(mt1);

        Pose2d newTarget = calculateTargetPose(mt1);
        if (newTarget != null) {
            targetPose = newTarget;
        }

        if (targetPose == null) {
            driveSubsystem.drive(0, 0, 0, true);
            return;
        }

        double xErr = targetPose.getX() - mt1.pose.getX();
        double yErr = targetPose.getY() - mt1.pose.getY();
        double rotErr = normalizeAngle(
            targetPose.getRotation().getDegrees() - mt1.pose.getRotation().getDegrees()
        );

        double xSpd = clamp(xPID.calculate(0, -xErr), -1.5, 1.5);
        double ySpd = clamp(yPID.calculate(0, -yErr), -1.5, 1.5);
        double rotSpd = clamp(rotPID.calculate(0, -rotErr), -90, 90);

        double distToTarget = Math.sqrt(xErr * xErr + yErr * yErr);
        if (distToTarget < 0.3) {
            xSpd *= 0.6;
            ySpd *= 0.6;
            rotSpd *= 0.7;
        }

        driveSubsystem.drive(xSpd, ySpd, Math.toRadians(rotSpd), true);

        boolean currentlyAligned = 
            Math.abs(xErr) < positionTolerance &&
            Math.abs(yErr) < positionTolerance &&
            Math.abs(rotErr) < rotationTolerance;

        if (currentlyAligned) {
            stableFrameCount++;
            if (stableFrameCount >= REQUIRED_STABLE_FRAMES) {
                aligned = true;
                System.out.println("[AUTO CLIMB] Aligned! Tags seen: " + climbTagCount);
            }
        } else {
            stableFrameCount = 0;
        }
    }

    private boolean isValidEstimate(PoseEstimate est) {
        if (est == null || est.tagCount == 0) {
            return false;
        }

        if (est.tagCount == 1 && est.rawFiducials.length == 1) {
            RawFiducial tag = est.rawFiducials[0];
            if (tag.ambiguity > SINGLE_TAG_MAX_AMBIGUITY) {
                return false;
            }
            if (tag.distToCamera > SINGLE_TAG_MAX_DIST) {
                return false;
            }
        }

        if (est.tagCount >= 2) {
            boolean hasCloseTag = false;
            for (RawFiducial tag : est.rawFiducials) {
                if (tag.distToCamera < MULTI_TAG_MAX_DIST) {
                    hasCloseTag = true;
                    break;
                }
            }
            if (!hasCloseTag) {
                return false;
            }
        }

        return true;
    }

    private int countClimbTags(PoseEstimate est) {
        int[] targetTags = getAllianceClimbTags();
        int count = 0;

        for (RawFiducial f : est.rawFiducials) {
            for (int id : targetTags) {
                if (f.id == id) {
                    count++;
                    break;
                }
            }
        }
        return count;
    }

    private int[] getAllianceClimbTags() {
        return DriverStation.getAlliance()
            .map(a -> a == Alliance.Red ? RED_CLIMB_TAGS : BLUE_CLIMB_TAGS)
            .orElse(BLUE_CLIMB_TAGS);
    }

    private Pose2d calculateTargetPose(PoseEstimate est) {
        int[] targetTags = getAllianceClimbTags();

        RawFiducial closestTag = null;
        double minDist = Double.MAX_VALUE;

        for (RawFiducial f : est.rawFiducials) {
            for (int id : targetTags) {
                if (f.id == id && f.distToCamera < minDist) {
                    minDist = f.distToCamera;
                    closestTag = f;
                }
            }
        }

        if (closestTag == null) {
            return null;
        }

        double angleToTagRad = Math.toRadians(closestTag.txnc);
        double distToMove = closestTag.distToCamera - targetDistance;

        Rotation2d targetRot = est.pose.getRotation()
            .plus(Rotation2d.fromDegrees(closestTag.txnc));

        Translation2d moveVector = new Translation2d(distToMove, 0)
            .rotateBy(est.pose.getRotation().plus(Rotation2d.fromRadians(angleToTagRad)));

        Translation2d targetTrans = est.pose.getTranslation().plus(moveVector);

        return new Pose2d(targetTrans, targetRot);
    }

    private double normalizeAngle(double degrees) {
        while (degrees > 180) degrees -= 360;
        while (degrees < -180) degrees += 360;
        return degrees;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    @Override
    public boolean isFinished() {
        return aligned;
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(0, 0, 0, true);

        if (interrupted) {
            System.out.println("[AUTO CLIMB] Alignment interrupted");
        } else {
            System.out.println("[AUTO CLIMB] Alignment complete - ready for climb!");
        }
    }
}

