// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import edu.wpi.first.networktables.BooleanPublisher;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.networktables.StringPublisher;
// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class AlmostDataManager extends SubsystemBase {
//   private AutonomousModeManager s_AutonomousModeManager;

//   // private DigitalInput DIm_ClimbLimitSwitch = new DigitalInput(0);
//   private DigitalInput DIm_IRSensor = new DigitalInput(1);
  
//   private SendableChooser<String> AutoSelection = new SendableChooser<String>();
  
//   private BooleanPublisher IRSensorTable;
//   private BooleanPublisher ClimbLimitSwitchTable;

//   private StringPublisher AutoModeDisplayTable;


//   /** Creates a new DataManager. If AutonomousModeManager is not set, please set it using the {@link #setAutonomousModeManager} */
//   public AlmostDataManager(AutonomousModeManager ts_AutonomousModeManager) {
//     s_AutonomousModeManager = ts_AutonomousModeManager;
    
//     NetworkTable Datatable = NetworkTableInstance.getDefault().getTable("datatable");

//     IRSensorTable = Datatable.getBooleanTopic("IR Sensor").publish();
//     ClimbLimitSwitchTable = Datatable.getBooleanTopic("Climb Switch").publish();

//     try {
//       AutoSelection.setDefaultOption("None", "N");
//       AutoSelection.addOption("Left", "L");
//       AutoSelection.addOption("Right", "R");

//       SmartDashboard.putData("Auto Mode Selector", AutoSelection);

//       AutoModeDisplayTable = Datatable.getStringTopic("Auto Mode Display").publish();

//       String Default = s_AutonomousModeManager.getAutonomousMode();
//       if (Default == null) {
//         Default = "None";
//       }
//       AutoModeDisplayTable.setDefault(Default);
//     } catch (Exception e) {
//       System.err.println("Error occured (Under normal circumstances, this should be fixed if you're going to use it. (Calling function again)): " + e);
//     }
//     //if (s_AutonomousModeManager != null) {
//     //  AutoSelection.setDefaultOption("None", "N");
//     //  AutoSelection.addOption("Left", "L");
//     //  AutoSelection.addOption("Right", "R");
// //
//     //  SmartDashboard.putData("Auto Mode Selector", AutoSelection);
// //
//     //  AutoModeDisplayTable = Datatable.getStringTopic("Auto Mode Display").publish();
//     //  AutoModeDisplayTable.setDefault(s_AutonomousModeManager.getAutonomousMode());
//     //}

//     System.out.println("Is Analog(?): " + DIm_IRSensor.isAnalogTrigger());
//   }

//   /** sets the depency for DataManager */
//   public void setAutonomousModeManager(AutonomousModeManager ts_AutonomousModeManager) {
//     s_AutonomousModeManager = ts_AutonomousModeManager;
//   }

//   // public boolean getClimbSwitch() {
//   //   return !DIm_ClimbLimitSwitch.get(); //inverted because when it is inactive it is true?
//   // }

//   public boolean getIRSenor() {
//     return DIm_IRSensor.get();
//   }

//   @Override
//   public void periodic() {
//     try {
//       switch (AutoSelection.getSelected()) {
//         case "L" -> s_AutonomousModeManager.putAutonomousMode(1);
//         case "R" -> s_AutonomousModeManager.putAutonomousMode(2);
//         default -> s_AutonomousModeManager.putAutonomousMode(0);
//       }

//       AutoModeDisplayTable.set(s_AutonomousModeManager.getAutonomousMode());
//     } catch (Exception e) {
//       System.err.println("Error occured (Under normal circumstances, this should be fixed if you're going to use it.): " + e);
//     }
//     //if (s_AutonomousModeManager != null) {
//     //  switch (AutoSelection.getSelected()) {
//     //    case "L" -> s_AutonomousModeManager.putAutonomousMode(1);
//     //    case "R" -> s_AutonomousModeManager.putAutonomousMode(2);
//     //    default -> s_AutonomousModeManager.putAutonomousMode(0);
//     //  }
// //
//     //  AutoModeDisplayTable.set(s_AutonomousModeManager.getAutonomousMode());
//     //}

//     // ClimbLimitSwitchTable.set(getClimbSwitch());
//     IRSensorTable.set(getIRSenor());
//   }
// }
