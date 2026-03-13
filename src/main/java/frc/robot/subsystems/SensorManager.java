// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import edu.wpi.first.networktables.BooleanPublisher;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class SensorManager extends SubsystemBase {
//   private DigitalInput DIm_IRSensor = new DigitalInput(1);
//   private BooleanPublisher IRSensorTable;
//   private boolean Sensor;

//   /** Creates a new SensorManager. */
//   public SensorManager() {
//     IRSensorTable = NetworkTableInstance.getDefault().getTable("datatable").getBooleanTopic("IR Sensor").publish();
//     System.out.println("Is Analog(?): " + DIm_IRSensor.isAnalogTrigger());
//   }

//   public boolean getIRSenor() {
//     return DIm_IRSensor.get();
//   }

//   @Override
//   public void periodic() {
//     Sensor = getIRSenor();
    
//     IRSensorTable.set(Sensor);
//     //System.out.println("Sensor(?): " + Sensor);
//   }
// }
