package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorFollowerConstants;
import frc.robot.Constants.MotorIDs;
import frc.robot.Constants.NeoMotorConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeSubsystem extends SubsystemBase {
  private final SparkMax leaderMotor = new SparkMax(MotorIDs.IntakePivotLeader, MotorType.kBrushless);
  private final RelativeEncoder encoder = leaderMotor.getEncoder();
  private final SparkMax followerMotor = new SparkMax(MotorIDs.IntakePivotFollower, MotorType.kBrushless);
  private final SparkMax intakeMotor = new SparkMax(MotorIDs.IntakeMotor, MotorType.kBrushless);

  // Soft limits
  private static final double FORWARD_SOFT_LIMIT = 0.425;
  private static final double REVERSE_SOFT_LIMIT = -0.01;

  // Motor speeds
  private static final double DEPLOY_SPEED = 0.1;
  private static final double STOW_SPEED = 0.1;

  // Holding voltage to fight gravity when stopped
  private static final double HOLD_VOLTAGE = -0.3; // Tune this value (start small!)

  public IntakeSubsystem() {
    configureMotors();

  }

  private void configureMotors() {
    SparkMaxConfig leaderConfig = new SparkMaxConfig();
    leaderConfig.encoder
        .positionConversionFactor(1.0 / 23.0)
        .velocityConversionFactor(1.0 / 23.0);
    leaderConfig.softLimit
        .forwardSoftLimit(FORWARD_SOFT_LIMIT)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimit(REVERSE_SOFT_LIMIT)
        .reverseSoftLimitEnabled(true);
    leaderConfig
        .smartCurrentLimit(NeoMotorConstants.IntakePivotCurrentLimit).idleMode(IdleMode.kBrake).closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    leaderConfig
        .inverted(false); // Adjust based on your testing

    leaderMotor.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig followerConfig = new SparkMaxConfig();
    followerConfig
        .smartCurrentLimit(NeoMotorConstants.IntakePivotCurrentLimit).idleMode(IdleMode.kBrake).closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    followerConfig
        .follow(leaderMotor, MotorFollowerConstants.IntakePivotFollowerIsOppositeOrientationAsLeader);

    followerMotor.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig intakeConfig = new SparkMaxConfig();
    intakeConfig
        .smartCurrentLimit(NeoMotorConstants.IntakeMotorCurrentLimit).idleMode(IdleMode.kCoast).closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    intakeConfig
        .inverted(false);

    intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // Zero encoder at startup - IMPORTANT: Arm must be stowed (up) when robot
    // starts!
    encoder.setPosition(0.0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("IntakePivot/Position (rotations)", getPosition());
    SmartDashboard.putNumber("IntakePivot/Position (degrees)", getPositionDegrees());
    SmartDashboard.putNumber("IntakePivot/Leader Current (A)",
        leaderMotor.getOutputCurrent());
    SmartDashboard.putNumber("IntakePivot/Applied Output",
        leaderMotor.getAppliedOutput());
    SmartDashboard.putBoolean("IntakePivot/At Forward Limit",
        getPosition() >= FORWARD_SOFT_LIMIT - 0.01);
    SmartDashboard.putBoolean("IntakePivot/At Reverse Limit",
        getPosition() <= REVERSE_SOFT_LIMIT + 0.01);
  }

  public void deployIntake() {
    leaderMotor.set(DEPLOY_SPEED);
  }

  public void stowIntake() {
    leaderMotor.set(-STOW_SPEED);
  }

  public void StopIntakePivot() { // Apply small upward voltage to hold against gravity
    leaderMotor.setVoltage(HOLD_VOLTAGE);
  }

  public double getPosition() {
    return encoder.getPosition();
  }

  public double getPositionDegrees() {
    return getPosition() * 360.0;
  }

  public void RunIntake() {
    intakeMotor.set(.8);
  }

  public void ReverseIntake() {
    intakeMotor.set(-.8);
  }

  public void StopIntake() {
    intakeMotor.set(0);
  }
}
