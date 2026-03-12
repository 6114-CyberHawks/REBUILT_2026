// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DIOPortIDs;
import frc.robot.Constants.MotorIDs;
import frc.robot.Constants.NeoMotorConstants;

// import frc.robot.util.Elastic;

public class ClimberSubsystem extends SubsystemBase {
  // Subsystem Constants
  private final SparkFlex ClimbMotor = new SparkFlex(MotorIDs.ClimbMotor, MotorType.kBrushless);
  private final RelativeEncoder Encoder = ClimbMotor.getEncoder();
  private final int Limit = 70; // 70 it rotations of output shaft, should be 1750 with gear ratio maybe
  private final int MaxHeight = 70;
  private final int MinHeight = 1;
  // private final double ManualSpeed = .7;
  private final double Speed = .8;

  private final DigitalInput LimitSwitch = new DigitalInput(DIOPortIDs.ClimbLimitSwitch);

  double positionTolerance = 2; // Can't be 0! May need to change based on speed
  double currentPosition;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    Config();
    Stop();
  }

  public void Config() {
    SparkFlexConfig ClimbConfig = new SparkFlexConfig();
    ClimbConfig
        .smartCurrentLimit(NeoMotorConstants.ClimbMotorCurrentLimit).idleMode(IdleMode.kBrake).closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    ClimbConfig
        .inverted(false);
        

    ClimbMotor.configure(ClimbConfig, SparkBase.ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
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

  public Command CommandStop() {
    return runOnce(() -> Stop());
  }

  public void Stop() {
    ClimbMotor.set(0);
  }

  public void SetSpeed(double speed){
    ClimbMotor.set(speed);
  }

  public Command Up() { // run motor positive
    return startEnd(
        () -> ClimbMotor.set(Speed),
        () -> Stop()
      ).until(() -> GetEncoderPosition() >= MaxHeight);
  }

  public Command Down() { // run motor negitive
    return startEnd(
        () -> ClimbMotor.set(-Speed),
        () -> Stop()
      ).until(() -> /*(GetEncoderPosition() <= MinHeight) || */ResetAtLimitSwitch());
  }

  public void ResetEncoder() {
    Encoder.setPosition(0);
  }

  public double GetEncoderPosition() {
    return Encoder.getPosition();
  }

  public boolean IsAtLimit() {
    currentPosition = GetEncoderPosition();
    if ((currentPosition >= (Limit - positionTolerance) && currentPosition <= (Limit + positionTolerance)) || !LimitSwitch.get())
      return true;
    if (currentPosition > Limit) {
      Stop();
      return true;
    } else
      return false;
  }

  public boolean ResetAtLimitSwitch() {
    if (!LimitSwitch.get()) {
      ResetEncoder();
      return true;
    }
    return false;
  }

  public void SetClimbPoint(int inputPosition) {
    currentPosition = GetEncoderPosition();
    if (!((currentPosition <= (inputPosition + positionTolerance)) && (currentPosition >= (inputPosition - positionTolerance)))) {
      System.out.println("not at limit: ");
      if(currentPosition < (inputPosition - positionTolerance)) {
        SetSpeed(Speed);
        System.out.println("Below input");
      } else if (currentPosition > (inputPosition - positionTolerance)) {
        System.out.println("Above input");
        SetSpeed(-Speed);
      }
    } else {
      System.out.println("between tolerance");
      Stop();
    }
    
  }
}
