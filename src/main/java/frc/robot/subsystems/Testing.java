// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.awt.Robot;
import java.util.concurrent.Flow.Publisher;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Testing extends SubsystemBase {
  DigitalInput DIm_IRSensor = new DigitalInput(9);

  /** Creates a new Testing. */
  public Testing() {
    System.out.println("Is Analog(?): " + DIm_IRSensor.isAnalogTrigger());
  }

  public boolean getIRSenor() {
    return DIm_IRSensor.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
