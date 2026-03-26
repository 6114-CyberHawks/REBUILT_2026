package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.vision.ShooterCalculator;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * Command that automatically adjusts shooter RPM and hood angle
 * based on distance to target using zone-based calculations.
 */
public class AutoAimCommand extends Command {

  private final VisionSubsystem visionSubsystem;
  private final HoodSubsystem hoodSubsystem;
  private final TurretSubsystem turretSubsystem;

  private ShooterCalculator.ShooterSettings targetSettings;
  private boolean isReady = false;

  public AutoAimCommand(VisionSubsystem vision, HoodSubsystem hood, TurretSubsystem turret) {
    this.visionSubsystem = vision;
    this.hoodSubsystem = hood;
    this.turretSubsystem = turret;

    addRequirements(hood, turret);
    // Note: Don't require vision - it should run in background
  }

  @Override
  public void initialize() {
    // Set vision to aim at center hub tags only
    visionSubsystem.aimAtCenterTagsOnly();

    isReady = false;
    SmartDashboard.putBoolean("AutoAim/Active", true);
  }

  @Override
  public void execute() {
    // Check if we have a valid target
    if (!visionSubsystem.hasValidTarget()) {
      System.out.println("case 1");
      SmartDashboard.putString("AutoAim/Status", "No Target");
      isReady = false;
      return;
    }

    // Get calculated shooter settings from vision
    targetSettings = visionSubsystem.getShooterSettings();

    if (!targetSettings.isValid) {
      System.out.println("case 2");

      SmartDashboard.putString("AutoAim/Status", "Invalid Settings");
      isReady = false;
      return;
    }

    // Apply settings to subsystems
    turretSubsystem.setRPM(targetSettings.rpm);
    hoodSubsystem.setPosition(targetSettings.hoodPosition);

    // Check if we're at target
    double currentRPM = turretSubsystem.getRPM();
    double currentHood = hoodSubsystem.getPosition();

    isReady = ShooterCalculator.isAtTarget(
        currentRPM, targetSettings.rpm,
        currentHood, targetSettings.hoodPosition);

    // Update dashboard
    SmartDashboard.putString("AutoAim/Status", isReady ? "READY" : "Adjusting");
    SmartDashboard.putNumber("AutoAim/Target RPM", targetSettings.rpm);
    SmartDashboard.putNumber("AutoAim/Target Hood", targetSettings.hoodPosition);
    SmartDashboard.putNumber("AutoAim/Distance", targetSettings.distance);
    SmartDashboard.putBoolean("AutoAim/Ready", isReady);
  }

  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("AutoAim/Active", false);
    SmartDashboard.putString("AutoAim/Status", "Inactive");
  }

  @Override
  public boolean isFinished() {
    // This command runs continuously until interrupted
    return false;
  }

  /**
   * Check if shooter is ready to fire
   */
  public boolean isReadyToShoot() {
    return hoodSubsystem.atTarget() && visionSubsystem.hasValidTarget();
  }
}
