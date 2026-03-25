package frc.robot.subsystems;


import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;
import frc.robot.vision.ShooterCalculator;
import frc.robot.vision.ScoringPosition;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class VisionSubsystem extends SubsystemBase {

    private final NetworkTable limelightTable;
    private final NetworkTableEntry tx;
    private final NetworkTableEntry ty;
    private final NetworkTableEntry tv;



    // Physical constants
    private static final double LIMELIGHT_HEIGHT_INCHES = 24.0;
    private static final double HUB_HEIGHT_INCHES = 44.25;
    private static final double LIMELIGHT_ANGLE_DEGREES = 25.0;

    // Tolerances
    private static final double DISTANCE_TOLERANCE_INCHES = 3.0;
    private static final double ANGLE_TOLERANCE_DEGREES = 2.0;

    // Scoring positions
    private final List<ScoringPosition> scoringPositions;
    private ScoringPosition currentTarget;
    private final SendableChooser<ScoringPosition> targetChooser;

    // Alliance tracking
    private Alliance currentAlliance = Alliance.Red;
    private boolean useManualAllianceOverride = false;

    // Aiming mode
    public enum AimingMode {
        CENTER_TAGS_ONLY,    // Only aim at center tags (one per hub side)
        ALL_HUB_TAGS,        // Aim at any hub tag
        SPECIFIC_TAG         // Aim at a specific tag ID
    }

    private AimingMode currentAimingMode = AimingMode.CENTER_TAGS_ONLY;
    private int specificTagID = -1;

    // Current shooter calculation
    private ShooterCalculator.ShooterSettings currentShooterSettings = 
        ShooterCalculator.ShooterSettings.invalid();

    public VisionSubsystem() {
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        tx = limelightTable.getEntry("tx");
        ty = limelightTable.getEntry("ty");
        tv = limelightTable.getEntry("tv");

        LimelightHelpers.SetFiducialDownscalingOverride("limelight", 1.0f);

        // Define scoring positions
        scoringPositions = new ArrayList<>();
        scoringPositions.add(new ScoringPosition("Close Shot", 60.0));
        scoringPositions.add(new ScoringPosition("Medium Shot", 100.0));
        scoringPositions.add(new ScoringPosition("Far Shot", 150.0));
        scoringPositions.add(new ScoringPosition("Max Range", 200.0));
        scoringPositions.add(new ScoringPosition("Left Angle - Close", 80.0, -20.0));
        scoringPositions.add(new ScoringPosition("Left Angle - Far", 120.0, -25.0));
        scoringPositions.add(new ScoringPosition("Right Angle - Close", 80.0, 20.0));
        scoringPositions.add(new ScoringPosition("Right Angle - Far", 120.0, 25.0));

        // Create target chooser
        targetChooser = new SendableChooser<>();
        targetChooser.setDefaultOption("Auto Select Closest", null);
        for (ScoringPosition pos : scoringPositions) {
            targetChooser.addOption(pos.getName(), pos);
        }
        SmartDashboard.putData("Shooting Position", targetChooser);

        // Alliance override for testing
        SmartDashboard.putBoolean("Vision/Force Red Alliance", false);
        SmartDashboard.putBoolean("Vision/Force Blue Alliance", false);

        currentTarget = scoringPositions.get(0);

        // Initialize with center tags for default alliance
        updateAllianceTagFilter();
    }

    @Override
    public void periodic() {
        // Update alliance detection
        updateAlliance();

        // Update AprilTag filter based on alliance and aiming mode
        updateAllianceTagFilter();

        // Update current target
        updateCurrentTarget();

        // Calculate shooter settings based on current distance
        if (hasValidTarget()) {
            double distance = getDistanceToHub();
            currentShooterSettings = ShooterCalculator.calculateFromDistance(distance);
        } else {
            currentShooterSettings = ShooterCalculator.ShooterSettings.invalid();
        }

        // Update SmartDashboard
        updateDashboard();
    }

    // ========================================
    // ALLIANCE MANAGEMENT
    // ========================================

    /**
     * Update current alliance from Driver Station or manual override
     */
    private void updateAlliance() {
        // Check for manual override (for testing)
        if (SmartDashboard.getBoolean("Vision/Force Red Alliance", false)) {
            currentAlliance = Alliance.Red;
            useManualAllianceOverride = true;
            return;
        }
        if (SmartDashboard.getBoolean("Vision/Force Blue Alliance", false)) {
            currentAlliance = Alliance.Blue;
            useManualAllianceOverride = true;
            return;
        }

        useManualAllianceOverride = false;

        // Get alliance from Driver Station
        Optional<Alliance> dsAlliance = DriverStation.getAlliance();
        if (dsAlliance.isPresent()) {
            currentAlliance = dsAlliance.get();
        }
        // If no alliance detected, keep previous value
    }

    /**
     * Get current alliance
     */
    public Alliance getCurrentAlliance() {
        return currentAlliance;
    }

    /**
     * Check if we're on Red alliance
     */
    public boolean isRedAlliance() {
        return currentAlliance == Alliance.Red;
    }

    // ========================================
    // APRILTAG FILTERING
    // ========================================

    /**
     * Update the Limelight's AprilTag filter based on alliance and aiming mode
     */
    private void updateAllianceTagFilter() {
        int[] tagsToTrack;

        switch (currentAimingMode) {
            case CENTER_TAGS_ONLY:
                tagsToTrack = getCenterHubTags();
                break;
            case ALL_HUB_TAGS:
                tagsToTrack = getAllHubTags();
                break;
            case SPECIFIC_TAG:
                tagsToTrack = (specificTagID > 0) ? new int[]{specificTagID} : getCenterHubTags();
                break;
            default:
                tagsToTrack = getCenterHubTags();
        }

        // Apply filter to Limelight
        LimelightHelpers.SetFiducialIDFiltersOverride("", tagsToTrack);
    }

    /**
     * Get center hub tags for current alliance (one per side)
     */
    public int[] getCenterHubTags() {
        if (isRedAlliance()) {
            return VisionConstants.RED_HUB_CENTER_TAGS;
        } else {
            return VisionConstants.BLUE_HUB_CENTER_TAGS;
        }
    }

    /**
     * Get all hub tags for current alliance
     */
    public int[] getAllHubTags() {
        if (isRedAlliance()) {
            return VisionConstants.ALLIANCE_HUB_TAG_ID_RED;
        } else {
            return VisionConstants.ALLIANCE_HUB_TAG_ID_BLUE;
        }
    }

    /**
     * Set aiming mode
     */
    public void setAimingMode(AimingMode mode) {
        this.currentAimingMode = mode;
        updateAllianceTagFilter();
    }

    /**
     * Set to aim at center tags only (RECOMMENDED)
     */
    public void aimAtCenterTagsOnly() {
        setAimingMode(AimingMode.CENTER_TAGS_ONLY);
    }

    /**
     * Set to aim at any hub tag
     */
    public void aimAtAnyHubTag() {
        setAimingMode(AimingMode.ALL_HUB_TAGS);
    }

    /**
     * Set to aim at a specific tag ID
     */
    public void aimAtSpecificTag(int tagID) {
        this.specificTagID = tagID;
        setAimingMode(AimingMode.SPECIFIC_TAG);
    }

    /**
     * Get the currently targeted AprilTag ID
     */
    public int getCurrentTargetTagID() {
        if (!hasValidTarget()) {
            return -1;
        }
        return (int) LimelightHelpers.getFiducialID("");
    }

    // ========================================
    // TARGET DETECTION & DISTANCE
    // ========================================

    /**
     * Check if we have a valid target
     */
    public boolean hasValidTarget() {
        return tv.getDouble(0) >= 1.0;
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
     * Calculate distance to hub using Limelight pose estimation
     */
    public double getDistanceToHub() {
        double[] pose = LimelightHelpers.getTargetPose_RobotSpace("");

        if (pose == null || pose.length < 3) {
            // Fallback to trigonometry method
            return getDistanceToHubTrig();
        }

        double x = pose[0]; // Side-to-side
        double z = pose[2]; // Forward/Backward

        double distance = Units.metersToInches(Math.sqrt(x * x + z * z));

        // Sanity check
        if (distance <= 0 || Double.isNaN(distance) || distance > 300) {
            return getDistanceToHubTrig();
        }

        return distance;
    }

    /**
     * Calculate distance using trigonometry (backup method)
     */
    private double getDistanceToHubTrig() {
        double verticalOffset = getVerticalOffset();
        double heightDifference = HUB_HEIGHT_INCHES - LIMELIGHT_HEIGHT_INCHES;
        double angleToTarget = LIMELIGHT_ANGLE_DEGREES + verticalOffset;

        if (angleToTarget <= 0) {
            return Double.MAX_VALUE;
        }

        return heightDifference / Math.tan(Math.toRadians(angleToTarget));
    }

    // ========================================
    // SHOOTER SETTINGS
    // ========================================

    /**
     * Get calculated shooter settings for current target
     */
    public ShooterCalculator.ShooterSettings getShooterSettings() {
        return currentShooterSettings;
    }

    /**
     * Get recommended RPM for current distance
     */
    public double getRecommendedRPM() {
        return currentShooterSettings.isValid ? currentShooterSettings.rpm : 
            Constants.ShooterConstants.DEFAULT_SHOOTER_RPM;
    }

    /**
     * Get recommended hood position for current distance
     */
    public double getRecommendedHoodPosition() {
        return currentShooterSettings.isValid ? currentShooterSettings.hoodPosition : 0.5;
    }

    // ========================================
    // SCORING POSITION MANAGEMENT
    // ========================================

    private ScoringPosition findClosestScoringPosition() {
        if (!hasValidTarget()) {
            return currentTarget;
        }

        double currentDistance = getDistanceToHub();
        double currentAngle = getHorizontalOffset();

        ScoringPosition closest = null;
        double minError = Double.MAX_VALUE;

        for (ScoringPosition pos : scoringPositions) {
            double distanceError = Math.abs(currentDistance - pos.getTargetDistance());
            double angleError = Math.abs(currentAngle - pos.getTargetAngle());
            double totalError = distanceError + (angleError * 2.0);

            if (totalError < minError) {
                minError = totalError;
                closest = pos;
            }
        }

        return closest != null ? closest : currentTarget;
    }

    private void updateCurrentTarget() {
        ScoringPosition selected = targetChooser.getSelected();
        if (selected == null) {
            currentTarget = findClosestScoringPosition();
        } else {
            currentTarget = selected;
        }
    }

    public ScoringPosition getCurrentTarget() {
        return currentTarget;
    }

    // ========================================
    // ALIGNMENT CHECKS
    // ========================================

    public boolean isAngleAligned() {
        if (!hasValidTarget()) return false;
        double error = Math.abs(getHorizontalOffset() - currentTarget.getTargetAngle());
        return error < ANGLE_TOLERANCE_DEGREES;
    }

    public boolean isDistanceCorrect() {
        if (!hasValidTarget()) return false;
        double distance = getDistanceToHub();
        double targetDistance = currentTarget.getTargetDistance();
        return Math.abs(distance - targetDistance) < DISTANCE_TOLERANCE_INCHES;
    }

    public boolean isReadyToScore() {
        return hasValidTarget() && isAngleAligned() && isDistanceCorrect();
    }

    public double getDistanceError() {
        if (!hasValidTarget()) return 0.0;
        return getDistanceToHub() - currentTarget.getTargetDistance();
    }

    public double getAngleError() {
        if (!hasValidTarget()) return 0.0;
        return getHorizontalOffset() - currentTarget.getTargetAngle();
    }

    public double getAlignmentQuality() {
        if (!hasValidTarget()) return 0.0;

        double distanceError = Math.abs(getDistanceError());
        double distanceQuality = Math.max(0, 1.0 - (distanceError / (DISTANCE_TOLERANCE_INCHES * 3)));

        double angleError = Math.abs(getAngleError());
        double angleQuality = Math.max(0, 1.0 - (angleError / (ANGLE_TOLERANCE_DEGREES * 3)));

        return (distanceQuality + angleQuality) / 2.0;
    }

    // ========================================
    // DASHBOARD
    // ========================================

    private void updateDashboard() {
        // Alliance info
        SmartDashboard.putString("Vision/Alliance", currentAlliance.toString());
        SmartDashboard.putBoolean("Vision/Alliance Override Active", useManualAllianceOverride);
        SmartDashboard.putString("Vision/Aiming Mode", currentAimingMode.toString());

        // Target info
        SmartDashboard.putBoolean("Vision/Target/Has Target", hasValidTarget());
        SmartDashboard.putNumber("Vision/Target/Target Tag ID", getCurrentTargetTagID());
        SmartDashboard.putNumber("Vision/Target/Distance (in)", getDistanceToHub());
        SmartDashboard.putNumber("Vision/Target/Angle Offset (deg)", getHorizontalOffset());

        // Shooter settings
        SmartDashboard.putBoolean("Vision/Shooter/Shooter Settings Valid", currentShooterSettings.isValid);
        SmartDashboard.putNumber("Vision/Shooter/Recommended RPM", getRecommendedRPM());
        SmartDashboard.putNumber("Vision/Shooter/Recommended Hood", getRecommendedHoodPosition());

        // Alignment
        SmartDashboard.putString("Vision/Current Position", currentTarget.getName());
        SmartDashboard.putNumber("Vision/Distance Error", getDistanceError());
        SmartDashboard.putNumber("Vision/Angle Error", getAngleError());
        SmartDashboard.putBoolean("Vision/Ready to Score", isReadyToScore());
        SmartDashboard.putNumber("Vision/Alignment Quality", getAlignmentQuality());
    }

    // ========================================
    // POSITION SELECTION
    // ========================================

    public void selectPosition(int index) {
        if (index >= 0 && index < scoringPositions.size()) {
            currentTarget = scoringPositions.get(index);
        }
    }

    public void cycleToNextPosition() {
        int currentIndex = scoringPositions.indexOf(currentTarget);
        int nextIndex = (currentIndex + 1) % scoringPositions.size();
        currentTarget = scoringPositions.get(nextIndex);
    }

    public void cycleToPreviousPosition() {
        int currentIndex = scoringPositions.indexOf(currentTarget);
        int prevIndex = (currentIndex - 1 + scoringPositions.size()) % scoringPositions.size();
        currentTarget = scoringPositions.get(prevIndex);
    }

    public List<ScoringPosition> getAllPositions() {
        return new ArrayList<>(scoringPositions);
    }
}

