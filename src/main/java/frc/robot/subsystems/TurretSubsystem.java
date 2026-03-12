// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DIOPortIDs;
import frc.robot.Constants.MotorFollowerConstants;
import frc.robot.Constants.MotorIDs;
import frc.robot.Constants.NeoMotorConstants;

public class TurretSubsystem extends SubsystemBase {
  private final SparkMax TurretShootLeader = new SparkMax(MotorIDs.TurretShootLeader, MotorType.kBrushless);
  private final AbsoluteEncoder TurretEncoder = TurretShootLeader.getAbsoluteEncoder();
  private final SparkClosedLoopController pidController = TurretShootLeader.getClosedLoopController();

  private final SparkMax TurretShootFollower1 = new SparkMax(MotorIDs.TurretShootFollower1, MotorType.kBrushless);
  private final SparkMax TurretShootFollower2 = new SparkMax(MotorIDs.TurretShootFollower2, MotorType.kBrushless);
  private final int topShootSpeed = 6050; // RPMs
  public static int ShootVelocity = 6000; // RPMs

  private final int BumpAmount = 50;

  private final SparkMax FeedTurret = new SparkMax(MotorIDs.FeedTurret, MotorType.kBrushless);
  public static double FeedSpeed = .9;

  // PID Constants - TUNE THESE VALUES
  private static final double kP = 0.0005;
  private static final double kI = 0.0;
  private static final double kD = 0.0;
  private static final double kFF = 0.00006;

  // private static boolean limelightAiming = false;
  // private String AimLocation;

  /** Creates a new TurretSubsystem. */
  public TurretSubsystem() {
    ConfigSparks();
  }

  public void ConfigSparks() {    
    SparkMaxConfig ShootLeaderConfig = new SparkMaxConfig(); // leader shooter
    ShootLeaderConfig
        .smartCurrentLimit(NeoMotorConstants.ShooterCurrentLimit).idleMode(IdleMode.kCoast).closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(kP)
        .i(kI)
        .d(kD)
        .velocityFF(kFF)
        .outputRange(-1, 1);
    ShootLeaderConfig.closedLoop
        .allowedClosedLoopError(2, ClosedLoopSlot.kSlot0)
        .maxMotion
            .cruiseVelocity(10000, ClosedLoopSlot.kSlot0)
            .maxAcceleration(3000, ClosedLoopSlot.kSlot0);

    SparkMaxConfig ShootFollower1Config = new SparkMaxConfig(); // follower shooter1
    ShootFollower1Config
        .smartCurrentLimit(NeoMotorConstants.ShooterCurrentLimit).idleMode(IdleMode.kCoast).closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(kP)
        .i(kI)
        .d(kD)
        .velocityFF(kFF)
        .outputRange(-1, 1);
    ShootFollower1Config
        .follow(TurretShootLeader, MotorFollowerConstants.TurretFollower1IsOppositeOrientationAsLeader);
    ShootFollower1Config.closedLoop
        .allowedClosedLoopError(2, ClosedLoopSlot.kSlot0)
        .maxMotion
            .cruiseVelocity(10000, ClosedLoopSlot.kSlot0)
            .maxAcceleration(3000, ClosedLoopSlot.kSlot0);

    SparkMaxConfig ShootFollower2Config = new SparkMaxConfig(); // follower shooter2
    ShootFollower2Config
        .smartCurrentLimit(NeoMotorConstants.ShooterCurrentLimit).idleMode(IdleMode.kCoast).closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(kP)
        .i(kI)
        .d(kD)
        .velocityFF(kFF)
        .outputRange(-1, 1);
    ShootFollower2Config
        .follow(TurretShootLeader, MotorFollowerConstants.TurretFollower2IsOppositeOrientationAsLeader);
    ShootFollower2Config.closedLoop
        .allowedClosedLoopError(2, ClosedLoopSlot.kSlot0)
        .maxMotion
            .cruiseVelocity(10000, ClosedLoopSlot.kSlot0)
            .maxAcceleration(3000, ClosedLoopSlot.kSlot0);

    SparkMaxConfig FeedConfig = new SparkMaxConfig(); // feed
    FeedConfig
        .smartCurrentLimit(NeoMotorConstants.ShooterCurrentLimit).idleMode(IdleMode.kCoast).closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    // keep motor config over powercycle
    TurretShootLeader.configure(ShootLeaderConfig, SparkBase.ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    TurretShootFollower1.configure(ShootFollower1Config, SparkBase.ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    TurretShootFollower2.configure(ShootFollower2Config, SparkBase.ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    FeedTurret.configure(FeedConfig, SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void DelayMS(int input) {
    try {
      Thread.sleep(input); // Sleep for 500 milliseconds (half a second)
    } catch (InterruptedException e) {
      // Handle the exception (e.g., log it or re-throw it)
      e.printStackTrace();
    }
  }

  public void FeedForward() {
    FeedTurret.set(FeedSpeed);
  }
  
  public void ReverseFeed() {
    FeedTurret.set(-.2);
  }

  public void StopFeed() {
    FeedTurret.set(0);
  }

  // Shooter
  public void IncreaseShootSpeedSpeed() {
    if (ShootVelocity < topShootSpeed)
      ShootVelocity += BumpAmount;
    else
      System.out.println("The Shoot speed speed is MAXED");
    System.out.println("increaseing Shoot Speed % is " + ShootVelocity);
    DelayMS(100);
  }

  public void DecreaseShootSpeedSpeed() {
    ShootVelocity -= BumpAmount;
    System.out.println("decreasing Shoot Speed % is " + ShootVelocity);
    DelayMS(100);
  }

  public void ShootFuel() {
    pidController.setReference(ShootVelocity, SparkMax.ControlType.kVelocity);
  }

  public void ReverseShooter() {
    TurretShootLeader.set(-.2);
  }

  public void StopShooter() {
    TurretShootLeader.set(0);
  }
}
