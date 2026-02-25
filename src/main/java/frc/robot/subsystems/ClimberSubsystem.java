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
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DIOPortIDs;
import frc.robot.Constants.MotorIDs;
import frc.robot.Constants.NeoMotorConstants;

public class ClimberSubsystem extends SubsystemBase {
  // Subsystem Constants
  private final SparkFlex ClimbMotor = new SparkFlex(MotorIDs.Climber, MotorType.kBrushless);
  private final RelativeEncoder Encoder = ClimbMotor.getEncoder();
  private final int ClimbHeight = 70;
  private final int MinHeight = 1;
  private final double ManualSpeed = .7;
  private final double Speed = .8;

  private final DigitalInput BeamBreak = new DigitalInput(DIOPortIDs.ClimbBeamBreakLimit);

  double positionTolerance = 2; // Can't be 0! May need to change based on speed
  double currentPosition;

  // Subsystem Variables
  private String Location;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    Config();
    Stop();
  }

  public void Config() {
    SparkFlexConfig ClimbConfig = new SparkFlexConfig();
    ClimbConfig
        .smartCurrentLimit(NeoMotorConstants.bigMotorCurrentLimit).idleMode(IdleMode.kBrake).closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    ClimbMotor.configure(ClimbConfig, SparkBase.ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (GetEncoderPosition() >= ClimbHeight) {
      Location = "Climb Hight";
    } else {
      Location = "Down";
    }
  }

  public void DelayMS(int input) {
    try {
      Thread.sleep(input); // Sleep for 500 milliseconds (half a second)
    } catch (InterruptedException e) {
      // Handle the exception (e.g., log it or re-throw it)
      e.printStackTrace();
    }
  }

  public void SetSpeed(double Speed) {
    ClimbMotor.set(Speed);
  }

  public Command ResetEncoder() {
    return this.runOnce(() -> Encoder.setPosition(70.0));
  }

  public double GetEncoderPosition() {
    return Encoder.getPosition() * -1;
  }

  public Command Up() {
    return runOnce(() -> {
      if (GetEncoderPosition() < ClimbHeight) {
        ClimbMotor.set(ManualSpeed);
      } else {
        Stop();
      }
    });
  }

  public Command Down() {
    return runOnce(() -> {
      if (GetEncoderPosition() > MinHeight/*!BeamBreak.get()*/) {
        ClimbMotor.set(-ManualSpeed);
      } else {
        Stop();
      }
    });
  }

  public Command CommandStop() {
    return runOnce(() -> Stop());
  }

  public void Stop() {
    ClimbMotor.set(0);
  }

  public void SetClimb(int inputPosition) {
    currentPosition = GetEncoderPosition();
    if (!((currentPosition <= (inputPosition + positionTolerance))
        && (currentPosition >= (inputPosition - positionTolerance)))) {
      System.out.println("Outside Bounds");
      if (currentPosition < (inputPosition - positionTolerance)) {
        SetSpeed(-Speed);
        System.out.println("Above Position");
      } else if (currentPosition > (inputPosition - positionTolerance)) {
        System.out.println("BelowPosition");
        SetSpeed(Speed);
      }
    } else {
      System.out.println("Within Bounds");
      Stop();
    }
  }
}
