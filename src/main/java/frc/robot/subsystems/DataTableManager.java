// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.StringPublisher;

public class DataTableManager extends SubsystemBase {
  private AutonomousModeManager s_AutonomousModeManager = new RobotContainer().s_AutonomousModeManager;

  private DigitalInput DIm_IRSensor = new DigitalInput(1);
  
  
  private BooleanPublisher IRSensorTable;
  private StringPublisher AutoModeTable;


  /** Creates a new SensorManager. */
  public DataTableManager() {

    IRSensorTable = NetworkTableInstance.getDefault().getTable("datatable").getBooleanTopic("IR Sensor").publish();
    AutoModeTable = NetworkTableInstance.getDefault().getTable("datatable").getStringTopic("Auto Mode").publish();
    System.out.println("Is Analog(?): " + DIm_IRSensor.isAnalogTrigger());
  }

  public boolean getIRSenor() {
    return DIm_IRSensor.get();
  }

  @Override
  public void periodic() {
    IRSensorTable.set(getIRSenor());
    AutoModeTable.set(s_AutonomousModeManager.getAutonomousMode());
  }
}
