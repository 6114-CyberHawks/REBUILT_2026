// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DIOPortIDs;
import frc.robot.Constants.MotorIDs;
import frc.robot.Constants.NeoMotorConstants;

public class TurretSubsystem extends SubsystemBase {
  private final SparkMax TurretShootLeader = new SparkMax(MotorIDs.TurretShoot1, MotorType.kBrushless);
  private final SparkMax TurretShootFollower1 = new SparkMax(MotorIDs.TurretShoot2, MotorType.kBrushless);
  private final SparkMax TurretShootFollower2 = new SparkMax(MotorIDs.TurretShoot3, MotorType.kBrushless);
  private final double topShootSpeed = .9;
  private static double ShootSpeed = 0.3;

  private final SparkMax FeedTurret = new SparkMax(MotorIDs.FeedTurret, MotorType.kBrushless);
  private static double FeedSpeed = 0.3;

  private final SparkMax TurretHood = new SparkMax(MotorIDs.TurretHood, MotorType.kBrushless);
  private final RelativeEncoder HoodEncoder = TurretHood.getEncoder(); // gear ratio is 1/3.2
  private final DigitalInput bottomLimitSwitch = new DigitalInput(DIOPortIDs.HoodBottomLimit);
  double PositionTolerance = 2; // Can't be 0!
  private final double MaxHoodAngle = 120;
  private final double MinHoodAngle = 0;
  private final double HoodSpeed = .3;
  private final double HoodGearing = 1 / 3.2;

  private static boolean limelightAiming = false;
  private String AimLocation;

  /** Creates a new TurretSubsystem. */
  public TurretSubsystem() {
    ConfigSparks();
  }

  public void ConfigSparks() {
    SparkMaxConfig ShootLeaderConfig = new SparkMaxConfig(); // leader shooter
    ShootLeaderConfig
        .smartCurrentLimit(NeoMotorConstants.bigMotorCurrentLimit).idleMode(IdleMode.kCoast).closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    SparkMaxConfig ShootFollowerConfig = new SparkMaxConfig(); // follower shooters
    ShootFollowerConfig
        .smartCurrentLimit(NeoMotorConstants.bigMotorCurrentLimit).idleMode(IdleMode.kCoast).closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    ShootFollowerConfig
        .follow(TurretShootLeader, true);

    SparkMaxConfig FeedConfig = new SparkMaxConfig(); // feed
    FeedConfig
        .smartCurrentLimit(NeoMotorConstants.bigMotorCurrentLimit).idleMode(IdleMode.kCoast).closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    SparkMaxConfig HoodConfig = new SparkMaxConfig(); // Hood
    HoodConfig
        .smartCurrentLimit(NeoMotorConstants.bigMotorCurrentLimit).idleMode(IdleMode.kBrake).closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    // keep motor config over powercycle
    TurretShootLeader.configure(ShootLeaderConfig, SparkBase.ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    TurretShootFollower1.configure(ShootFollowerConfig, SparkBase.ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    TurretShootFollower2.configure(ShootFollowerConfig, SparkBase.ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    FeedTurret.configure(FeedConfig, SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    TurretHood.configure(HoodConfig, SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // feeding
  public void IncreaseFeedSpeed() {
    if (ShootSpeed < topShootSpeed)
      FeedSpeed += .01;
    else
      System.out.println("The feed speed is MAXED");
  }

  public void DecreaseFeedSpeed() {
    FeedSpeed -= .01;
  }

  public void FastFeed() {
    FeedTurret.set(FeedSpeed);
  }

  // public void SlowFeed() {
  // FeedTurret.set(FeedSpeed2);
  // }

  public void ReverseFeed() {
    FeedTurret.set(-FeedSpeed);
  }

  public void StopFeed() {
    FeedTurret.set(0);
  }

  // Shooter
  public void IncreaseShootSpeedSpeed() {
    ShootSpeed += .01;
  }

  public void DecreaseShootSpeedSpeed() {
    ShootSpeed -= .01;
  }

  // public void SetShootSpeed(double shootSpeed) {
  // ShootSpeed = shootSpeed;
  // }

  public void ShootFuel() {
    TurretShootLeader.set(ShootSpeed);
  }

  public void StopShooter() {
    TurretShootLeader.set(0);
  }

  // hood
  public Command ResetEncoder() {
    return this.runOnce(() -> HoodEncoder.setPosition(0.0));
  }

  public Command Forward() {
    return this.runOnce(() -> {
      if (GetHoodPosition() < MaxHoodAngle)
        TurretHood.set(-HoodSpeed);
      else
        Stop();
    });
  }

  public Command Reverse() {
    return this.runOnce(() -> {
      if (bottomLimitSwitch.get())
        TurretHood.set(HoodSpeed);
      else
        Stop();
    });
  }

  public Command CommandStop() {
    return this.runOnce(() -> Stop());
  }

  public void Stop() {
    TurretHood.set(0);
  }

  public double GetHoodPosition() { // gets the angle the hood is at
    return (HoodEncoder.getPosition() / 3.2) * 360;
  }
}
