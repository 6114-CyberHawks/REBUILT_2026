// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DataTableManager extends SubsystemBase {
  private AutonomousModeManager s_AutonomousModeManager;

  private DigitalInput DIm_IRSensor = new DigitalInput(1);
  
  private SendableChooser<String> AutoSelection = new SendableChooser<String>();
  
  private BooleanPublisher IRSensorTable;
  //rivate BooleanPublisher AutoNoneModeTable;
  //rivate BooleanPublisher AutoLeftModeTable;
  //rivate BooleanPublisher AutoRightModeTable;
  //private StringPublisher AutoModeTable;
  private StringPublisher AutoModeDisplayTable;

  private StringSubscriber AutoModeTopic;


  /** Creates a new SensorManager. */
  public DataTableManager(AutonomousModeManager ts_AutonomousModeManager) {
    s_AutonomousModeManager = ts_AutonomousModeManager;

    AutoSelection.setDefaultOption("None", "N");
    AutoSelection.addOption("Left", "L");
    AutoSelection.addOption("Right", "R");

    SmartDashboard.putData("Auto Mode Selector", AutoSelection);

    NetworkTable Datatable = NetworkTableInstance.getDefault().getTable("datatable");
    IRSensorTable = Datatable.getBooleanTopic("IR Sensor").publish();
    //AutoNoneModeTable = NetworkTableInstance.getDefault().getTable("datatable").getBooleanTopic("Auto Mode None").publish();
    //AutoLeftModeTable = NetworkTableInstance.getDefault().getTable("datatable").getBooleanTopic("Auto Mode Left").publish();
    //AutoRightModeTable = NetworkTableInstance.getDefault().getTable("datatable").getBooleanTopic("Auto Mode Right").publish();
    //AutoModeTable = Datatable.getStringTopic("Auto Mode").publish();
    AutoModeDisplayTable = Datatable.getStringTopic("Auto Mode Display").publish();
    AutoModeTopic = Datatable.getStringTopic("Auto Mode").subscribe("None");
    AutoModeDisplayTable.setDefault(s_AutonomousModeManager.getAutonomousMode());


    System.out.println("Is Analog(?): " + DIm_IRSensor.isAnalogTrigger());
  }

  public boolean getIRSenor() {
    return DIm_IRSensor.get();
  }

  @Override
  public void periodic() {
    switch (AutoSelection.getSelected()) {
      case "L" -> s_AutonomousModeManager.putAutonomousMode(1);
      case "R" -> s_AutonomousModeManager.putAutonomousMode(2);
      default -> s_AutonomousModeManager.putAutonomousMode(0);
    }

    AutoModeDisplayTable.set(s_AutonomousModeManager.getAutonomousMode());
    //AutoNoneModeTable.set(s_AutonomousModeManager.getAutonomousMode().equals("None") ? true : false);
    //AutoLeftModeTable.set(s_AutonomousModeManager.getAutonomousMode().equals("Left") ? true : false);
    //AutoRightModeTable.set(s_AutonomousModeManager.getAutonomousMode().equals("Right") ? true : false);

    IRSensorTable.set(getIRSenor());
  }
}
