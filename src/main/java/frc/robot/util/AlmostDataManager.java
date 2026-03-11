// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlmostDataManager extends SubsystemBase {
  private AutonomousModeManager s_AutonomousModeManager;

  private DigitalInput DIm_ClimbLimitSwitch = new DigitalInput(0);
  private DigitalInput DIm_IRSensor = new DigitalInput(1);
  
  private SendableChooser<String> AutoSelection = new SendableChooser<String>();
  
  private BooleanPublisher IRSensorTable;
  private BooleanPublisher ClimbLimitSwitchTable;

  private StringPublisher AutoModeDisplayTable;

  public final Elastic.Notification notificationNullDepError = new Elastic.Notification(Elastic.NotificationLevel.ERROR, "Data Manager Error", "Autonomous Depecency is null.", 10000);
  public final Elastic.Notification notificationNullAutonModeError = new Elastic.Notification(Elastic.NotificationLevel.ERROR, "Auton Manager Error", "Autonomous is null.", 10000);
  public final Elastic.Notification notificationDepInfo = new Elastic.Notification(Elastic.NotificationLevel.INFO, "Data Manager Info", "Autonomous Depecency is declared", 10000);

  //public final Elastic.Notification notificationTestInfo = new Elastic.Notification(Elastic.NotificationLevel.INFO, "Notification test: INFO", "Notification Test...", 5000);
  //public final Elastic.Notification notificationTestWarning = new Elastic.Notification(Elastic.NotificationLevel.WARNING, "Notification test: WARNING", "Notification Test...", 5000);
  //public final Elastic.Notification notificationTestError = new Elastic.Notification(Elastic.NotificationLevel.ERROR, "Notification test: ERROR", "Notification Test...", 5000);

  /** Creates a new DataManager. If AutonomousModeManager is not set, please set it using the {@link #setAutonomousModeManager} */
  public AlmostDataManager(AutonomousModeManager ts_AutonomousModeManager) {
    s_AutonomousModeManager = ts_AutonomousModeManager;
    
    NetworkTable Datatable = NetworkTableInstance.getDefault().getTable("datatable");

    IRSensorTable = Datatable.getBooleanTopic("IR Sensor").publish();
    ClimbLimitSwitchTable = Datatable.getBooleanTopic("Climb Switch").publish();

    try {
      AutoSelection.setDefaultOption("None", "N");
      AutoSelection.addOption("Left", "L");
      AutoSelection.addOption("Right", "R");

      SmartDashboard.putData("Auto Mode Selector", AutoSelection);

      AutoModeDisplayTable = Datatable.getStringTopic("Auto Mode Display").publish();

      String Default = s_AutonomousModeManager.getAutonomousMode();
      if (Default == null) {
        Default = "None";
      }
      AutoModeDisplayTable.setDefault(Default);
    } catch (Exception e) {
      Elastic.sendNotification(notificationNullDepError);
      System.out.println("Error occured (Under normal circumstances, this should be fixed if you're going to use it.): " + e);
    }
    //if (s_AutonomousModeManager != null) {
    //  AutoSelection.setDefaultOption("None", "N");
    //  AutoSelection.addOption("Left", "L");
    //  AutoSelection.addOption("Right", "R");
//
    //  SmartDashboard.putData("Auto Mode Selector", AutoSelection);
//
    //  AutoModeDisplayTable = Datatable.getStringTopic("Auto Mode Display").publish();
    //  AutoModeDisplayTable.setDefault(s_AutonomousModeManager.getAutonomousMode());
    //}

    System.out.println("Is Analog(?): " + DIm_IRSensor.isAnalogTrigger());
  }

  /** sets the depency for DataManager */
  public void setAutonomousModeManager(AutonomousModeManager ts_AutonomousModeManager) {
    this.s_AutonomousModeManager = ts_AutonomousModeManager;
    sendNotification(notificationDepInfo);
  }

  public void sendNotification(Elastic.Notification Notification) {
    Elastic.sendNotification(Notification);
  }

  public boolean getClimbSwitch() {
    return !DIm_ClimbLimitSwitch.get(); //inverted because when it is inactive it is true?
  }

  public boolean getIRSenor() {
    return DIm_IRSensor.get();
  }

  @Override
  public void periodic() {
    try {
      switch (AutoSelection.getSelected()) {
        case "L" -> s_AutonomousModeManager.putAutonomousMode(1);
        case "R" -> s_AutonomousModeManager.putAutonomousMode(2);
        default -> s_AutonomousModeManager.putAutonomousMode(0);
      }

      AutoModeDisplayTable.set(s_AutonomousModeManager.getAutonomousMode());
    } catch (Exception e) {
      Elastic.sendNotification(notificationNullDepError);
      System.err.println("Error occured (Under normal circumstances, this should be fixed if you're going to use it.): " + e);
    }
    //if (s_AutonomousModeManager != null) {
    //  switch (AutoSelection.getSelected()) {
    //    case "L" -> s_AutonomousModeManager.putAutonomousMode(1);
    //    case "R" -> s_AutonomousModeManager.putAutonomousMode(2);
    //    default -> s_AutonomousModeManager.putAutonomousMode(0);
    //  }
//
    //  AutoModeDisplayTable.set(s_AutonomousModeManager.getAutonomousMode());
    //}

    ClimbLimitSwitchTable.set(getClimbSwitch());
    IRSensorTable.set(getIRSenor());
  }
}
