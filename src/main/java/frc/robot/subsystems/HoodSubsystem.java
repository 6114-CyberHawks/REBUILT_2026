package frc.robot.subsystems;

import java.lang.annotation.Target;

import org.opencv.highgui.HighGui;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorIDs;
import frc.robot.Constants.NeoMotorConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.TurretSubsystem;

public class HoodSubsystem extends SubsystemBase {

  // Hardware
  private final SparkMax hoodMotor = new SparkMax(MotorIDs.HoodMotor, MotorType.kBrushless);
  private final AbsoluteEncoder hoodEncoder = hoodMotor.getAbsoluteEncoder();
  private final SparkClosedLoopController pidController = hoodMotor.getClosedLoopController();

  // Constants
  private final double manualSpeed = .3;

  // Encoder positions (in rotations of the gearbox output shaft)
  private static final double MIN_POSITION = 0.01; // TODO: find lowest wanted position
  private static final double MAX_POSITION = 0.4;

  // Preset positions - in absolute encoder positions TODO: Find updated locations for each point
  private static final double LOW_POSITION = 0.0;
  private static final double MID_POSITION = 0.075;
  private static final double HIGH_POSITION = 0.15; // TODO: figure out actual location
  private static final double Pass_POSITION = 0.365;

  // gear ratio on motor to absolute encoder
  private static final double GEAR_RATIO = 16;

  // Incremental adjustment
  private static final double INCREMENT_AMOUNT = 0.01; // Adjust as needed

  // PID Constants - TUNE THESE VALUES
  private static final double kP = 0.5;
  private static final double kI = 0.0;
  private static final double kD = 0.0;
  private static final double kFF = 0.0;

  // Tolerance for position control
  private static final double POSITION_TOLERANCE = 0.005; // rotations

  // Target position
  private double targetPosition = LOW_POSITION;

  public HoodSubsystem() {
    // Set initial target
    targetPosition = getPosition();
  }

  public void config() {
    SparkMaxConfig config = new SparkMaxConfig();
    config
        .smartCurrentLimit(NeoMotorConstants.HoodMotorCurrentLimit).idleMode(IdleMode.kBrake).closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .p(kP)
        .i(kI)
        .d(kD)
        .velocityFF(kFF)
        .outputRange(-1, 1);

    config.softLimit
        .forwardSoftLimit(MAX_POSITION)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimit(MIN_POSITION)
        .reverseSoftLimitEnabled(true);

    config.closedLoop
        .allowedClosedLoopError(.005, ClosedLoopSlot.kSlot0)
        .maxMotion
            .cruiseVelocity(100, ClosedLoopSlot.kSlot0)
            .maxAcceleration(10, ClosedLoopSlot.kSlot0);

    config.disableVoltageCompensation();

    config.absoluteEncoder.inverted(false)
        .positionConversionFactor(1)
        .velocityConversionFactor(1)
        .zeroOffset(0);

  }

  @Override
  public void periodic() {
    // Update telemetry
    SmartDashboard.putNumber("Hood/Current Position", getPosition());
    SmartDashboard.putNumber("Hood/Target Position", targetPosition);
    SmartDashboard.putBoolean("Hood/At Target", atTarget());
    SmartDashboard.putNumber("Hood/Motor Current", hoodMotor.getOutputCurrent());
    SmartDashboard.putNumber("Hood/Motor Output", hoodMotor.getOutputCurrent());
    SmartDashboard.putNumber("Hood/Position error", targetPosition - getPosition());
  }

  // /**
  //  * Set Hood position to position higher than current position
  //  */
  // public void setNextPosition() {
  //   if (targetPosition == LOW_POSITION)
  //     setPosition(MID_POSITION);
  //   else if (targetPosition == MID_POSITION)
  //     setPosition(HIGH_POSITION);
  //   else if (targetPosition == HIGH_POSITION)
  //   System.out.println("Position Maxed");
  // }

  // /**
  //  * Set Hood position to position lower than current position
  //  */
  // public void setPreviousPosition() {
  //   if (targetPosition == HIGH_POSITION)
  //     setPosition(MID_POSITION);
  //   else if (targetPosition == MID_POSITION)
  //     setPosition(LOW_POSITION);
  //   else if (targetPosition == LOW_POSITION)
  //   System.out.println("Position Min-ed");
  // }

  /**
   * Set hood to a specific position
   * 
   * @param position Target position in encoder rotations (0.0 to 0.3)
   */
  public void setPosition(double position) {
    // Clamp position to valid range
    // 0 .15 .4
    targetPosition = Math.max(MIN_POSITION, Math.min(MAX_POSITION, position));

    System.out.println("Hood setPosition called: requested=" + position + "Clmaped=" + targetPosition);

    pidController.setReference(targetPosition * GEAR_RATIO, SparkMax.ControlType.kPosition);
  }

  /**
   * Set hood to low preset position
   */
  public void setLowPosition() {
    setPosition(LOW_POSITION);
    TurretSubsystem.ShootVelocity = 3800;
    TurretSubsystem.FeedSpeed = .9;
  }

  /**
   * Set hood to mid preset position
   */
  public void setMidPosition() {
    setPosition(MID_POSITION);
    TurretSubsystem.ShootVelocity = 3400;
    TurretSubsystem.FeedSpeed = .7;
  }

  /**
   * Set hood to high preset position
   */
  public void setHighPosition() {
    setPosition(HIGH_POSITION);
    TurretSubsystem.ShootVelocity = 2900;
    TurretSubsystem.FeedSpeed = .5;
  }

  /**
   * Set hood to Pass preset position
   */
  public void setPassPosition() {
    setPosition(Pass_POSITION);
    TurretSubsystem.ShootVelocity = 2500;
    TurretSubsystem.FeedSpeed = .9;
  }

  /**
   * Increment hood position by a small amount
   */
  public void incrementPosition() {
    setPosition(targetPosition + INCREMENT_AMOUNT);
  }

  /**
   * Decrement hood position by a small amount
   */
  public void decrementPosition() {
    setPosition(targetPosition - INCREMENT_AMOUNT);
  }

  /**
   * Get current hood position from encoder
   * 
   * @return Current position in encoder rotations
   */
  public double getPosition() {
    return hoodEncoder.getPosition();
  }

  /**
   * Get current target position
   * 
   * @return Target position in encoder rotations
   */
  public double getTargetPosition() {
    return targetPosition;
  }

  /**
   * Check if hood is at target position
   * 
   * @return true if within tolerance of target
   */
  public boolean atTarget() {
    return Math.abs(getPosition() - targetPosition) < POSITION_TOLERANCE;
  }

  /**
   * Stop the hood motor
   */
  public void stop() {
    hoodMotor.stopMotor();
  }

  /**
   * Manual control for testing (use with caution)
   * 
   * @param speed Speed from -1.0 to 1.0
   */
  public void setManualSpeed(double speed) {
    hoodMotor.set(speed);
  }
}
