// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorIDs;
import frc.robot.Constants.NeoMotorConstants;

public class HopperSubsystem extends SubsystemBase {
  private final SparkMax HopperMotor = new SparkMax(MotorIDs.HopperMotor, MotorType.kBrushless);
  private final double HighSpeed = .8;
  private final double LowSpeed = .4;

  /** Creates a new HopperSubsystem. */
  public HopperSubsystem() {
    config();
  }

  public void config(){
    SparkMaxConfig config = new SparkMaxConfig();
    config
      .smartCurrentLimit(NeoMotorConstants.HopperMotorCurrentLimit).idleMode(IdleMode.kCoast).closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    

      HopperMotor.configure(config, SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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

  public void Forward() {
    HopperMotor.set(HighSpeed);
  }

  public void Reverse() {
    HopperMotor.set(-LowSpeed);
  }

  public void Ocilate() {
    HopperMotor.set(HighSpeed);
    DelayMS(500);
    HopperMotor.set(LowSpeed);
    DelayMS(500);
  }

  public void Stop() {
    HopperMotor.set(0);
  }
}