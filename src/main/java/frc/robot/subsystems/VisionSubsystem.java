package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.vision.ScoringPosition;

import java.util.ArrayList;
import java.util.List;

public class VisionSubsystem extends SubsystemBase {
    private final NetworkTable limelightTable;
    private final NetworkTableEntry tx;  // Horizontal offset
    private final NetworkTableEntry ty;  // Vertical offset
    private final NetworkTableEntry tv;  // Valid target

    // limelight TargetPose in BotSpace
    private double[] pose;
    
    
        // Physical constants - MEASURE THESE ON YOUR ROBOT!
        private static final double LIMELIGHT_HEIGHT_INCHES = 24.0;
        private static final double HUB_HEIGHT_INCHES = 44.25;  // From game manual
        private static final double LIMELIGHT_ANGLE_DEGREES = 25.0;
    
        // Tolerances
        private static final double DISTANCE_TOLERANCE_INCHES = 3.0;
        private static final double ANGLE_TOLERANCE_DEGREES = 2.0;
    
        // Scoring positions - define your shooting positions here
        private final List<ScoringPosition> scoringPositions;
        private ScoringPosition currentTarget;
        private final SendableChooser<ScoringPosition> targetChooser;
    
        public VisionSubsystem() {
            limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
            tx = limelightTable.getEntry("tx");
            ty = limelightTable.getEntry("ty");
            tv = limelightTable.getEntry("tv");
    
            // Define your shooting positions
            // Format: new ScoringPosition("Name", distance_in_inches, angle_offset_degrees)
            scoringPositions = new ArrayList<>();
    
            // Straight-on shots at different distances
            scoringPositions.add(new ScoringPosition("Close Shot", 60.0));
            scoringPositions.add(new ScoringPosition("Medium Shot", 100.0));
            scoringPositions.add(new ScoringPosition("Far Shot", 150.0));
            scoringPositions.add(new ScoringPosition("Max Range", 200.0));
    
            // Angled shots (useful when defense is blocking center)
            scoringPositions.add(new ScoringPosition("Left Angle - Close", 80.0, -20.0));
            scoringPositions.add(new ScoringPosition("Left Angle - Far", 120.0, -25.0));
            scoringPositions.add(new ScoringPosition("Right Angle - Close", 80.0, 20.0));
            scoringPositions.add(new ScoringPosition("Right Angle - Far", 120.0, 25.0));
    
            // Create target chooser for driver station
            targetChooser = new SendableChooser<>();
            targetChooser.setDefaultOption("Auto Select Closest", null);
            for (ScoringPosition pos : scoringPositions) {
                targetChooser.addOption(pos.getName(), pos);
            }
            SmartDashboard.putData("Shooting Position", targetChooser);
    
            // Default to close shot
            currentTarget = scoringPositions.get(0);
        }
    
        @Override
        public void periodic() {
            updateCurrentTarget();
    
            // Update SmartDashboard for debugging
            SmartDashboard.putBoolean("Vision/Target/Has Target", hasValidTarget());
            SmartDashboard.putNumber("Vision/Distance/Distance (in) TargetPose_RobotSpace", getDistanceToHub());
            SmartDashboard.putNumber("Vision/Angle/Angle Offset (deg)", getHorizontalOffset());
            SmartDashboard.putString("Vision/Current Position", currentTarget.getName());
            SmartDashboard.putNumber("Vision/Target/Target Distance", currentTarget.getTargetDistance());
            SmartDashboard.putNumber("Vision/Target/Target Angle", currentTarget.getTargetAngle());
            SmartDashboard.putNumber("Vision/Distance/Distance Error", getDistanceError());
            SmartDashboard.putNumber("Vision/Angle/Angle Error", getAngleError());
            SmartDashboard.putBoolean("Vision/Ready to Score", isReadyToScore());
            SmartDashboard.putBoolean("Vision/Angle/Angle Aligned", isAngleAligned());
            SmartDashboard.putBoolean("Vision/Distance/Distance Correct", isDistanceCorrect());
            SmartDashboard.putNumber("Vision/Alignment Quality", getAlignmentQuality());
        }
    
        /**
         * Check if we have a valid target
         */
        public boolean hasValidTarget() {
            return tv.getDouble(0) == 1.0;
        }
    
        /**
         * Get horizontal offset from target in degrees
         */
        public double getHorizontalOffset() {
            return tx.getDouble(0.0);
        }
    
        /**
         * Get vertical offset from target in degrees
         */
        public double getVerticalOffset() {
            return ty.getDouble(0.0);
        }
    
        /**
         * Calculate distance to hub using trigonometry
         */
        public double getDistanceToHub() {
            pose = LimelightHelpers.getTargetPose_RobotSpace("limelight");
        double verticalOffset = getVerticalOffset();
        double heightDifference = HUB_HEIGHT_INCHES - LIMELIGHT_HEIGHT_INCHES;
        double angleToTarget = LIMELIGHT_ANGLE_DEGREES + verticalOffset;
        double x = pose[0]; // Side-to-side
        double z = pose[2]; // Forward/Backward


        // Avoid division by zero or negative angles
        if (angleToTarget <= 0) {
            return Double.MAX_VALUE;
        }

        double distance = Units.metersToInches(Math.sqrt(x * x + z * z));
        return distance;
    }

    /**
     * Find the closest scoring position based on current distance and angle
     */
    private ScoringPosition findClosestScoringPosition() {
        if (!hasValidTarget()) {
            return currentTarget; // Keep current if no target
        }

        double currentDistance = getDistanceToHub();
        double currentAngle = getHorizontalOffset();

        ScoringPosition closest = null;
        double minError = Double.MAX_VALUE;

        for (ScoringPosition pos : scoringPositions) {
            // Calculate total error from this position
            double distanceError = Math.abs(currentDistance - pos.getTargetDistance());
            double angleError = Math.abs(currentAngle - pos.getTargetAngle());

            // Weight distance error more heavily (you can adjust these weights)
            double totalError = distanceError + (angleError * 2.0);

            if (totalError < minError) {
                minError = totalError;
                closest = pos;
            }
        }

        return closest != null ? closest : currentTarget;
    }

    /**
     * Update the current target based on driver selection or auto-selection
     */
    private void updateCurrentTarget() {
        ScoringPosition selected = targetChooser.getSelected();

        if (selected == null) {
            // Auto-select mode - find closest position
            currentTarget = findClosestScoringPosition();
        } else {
            // Manual selection mode
            currentTarget = selected;
        }
    }

    /**
     * Get the current scoring position being targeted
     */
    public ScoringPosition getCurrentTarget() {
        return currentTarget;
    }

    /**
     * Check if robot is aligned with current target (angle)
     */
    public boolean isAngleAligned() {
        if (!hasValidTarget()) return false;

        double targetAngle = currentTarget.getTargetAngle();
        double currentAngle = getHorizontalOffset();
        double error = Math.abs(currentAngle - targetAngle);

        return error < ANGLE_TOLERANCE_DEGREES;
    }

    /**
     * Check if robot is at correct distance for current target
     */
    public boolean isDistanceCorrect() {
        if (!hasValidTarget()) return false;

        double distance = getDistanceToHub();
        double targetDistance = currentTarget.getTargetDistance();

        return Math.abs(distance - targetDistance) < DISTANCE_TOLERANCE_INCHES;
    }

    /**
     * Check if robot is ready to score (both angle and distance correct)
     */
    public boolean isReadyToScore() {
        return hasValidTarget() && isAngleAligned() && isDistanceCorrect();
    }

    /**
     * Get how far off we are from target distance (positive = too far, negative = too close)
     */
    public double getDistanceError() {
        if (!hasValidTarget()) return 0.0;
        return getDistanceToHub() - currentTarget.getTargetDistance();
    }

    /**
     * Get how far off we are from target angle (positive = too far right, negative = too far left)
     */
    public double getAngleError() {
        if (!hasValidTarget()) return 0.0;
        return getHorizontalOffset() - currentTarget.getTargetAngle();
    }

    /**
     * Get alignment quality (0.0 to 1.0, where 1.0 is perfect)
     */
    public double getAlignmentQuality() {
        if (!hasValidTarget()) {
            return 0.0;
        }

        // Calculate distance quality (1.0 at perfect, 0.0 at 3x tolerance away)
        double distanceError = Math.abs(getDistanceError());
        double distanceQuality = Math.max(0, 1.0 - (distanceError / (DISTANCE_TOLERANCE_INCHES * 3)));

        // Calculate angle quality (1.0 at perfect, 0.0 at 3x tolerance away)
        double angleError = Math.abs(getAngleError());
        double angleQuality = Math.max(0, 1.0 - (angleError / (ANGLE_TOLERANCE_DEGREES * 3)));

        // Average the two (you can weight differently if needed)
        return (distanceQuality + angleQuality) / 2.0;
    }

    /**
     * Manually select a specific scoring position by index
     */
    public void selectPosition(int index) {
        if (index >= 0 && index < scoringPositions.size()) {
            currentTarget = scoringPositions.get(index);
        }
    }

    /**
     * Cycle to the next scoring position
     */
    public void cycleToNextPosition() {
        int currentIndex = scoringPositions.indexOf(currentTarget);
        int nextIndex = (currentIndex + 1) % scoringPositions.size();
        currentTarget = scoringPositions.get(nextIndex);
    }

    /**
     * Cycle to the previous scoring position
     */
    public void cycleToPreviousPosition() {
        int currentIndex = scoringPositions.indexOf(currentTarget);
        int prevIndex = (currentIndex - 1 + scoringPositions.size()) % scoringPositions.size();
        currentTarget = scoringPositions.get(prevIndex);
    }

    /**
     * Get list of all available positions (for dashboard display)
     */
    public List<ScoringPosition> getAllPositions() {
        return new ArrayList<>(scoringPositions);
    }
}