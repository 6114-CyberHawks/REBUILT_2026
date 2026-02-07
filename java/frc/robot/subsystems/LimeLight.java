 // Copyright (c) FIRST and other WPILib contributors.
 // Open Source Software; you can modify and/or share it under the terms of
 // the WPILib BSD license file in the root directory of this project.

 package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimeLight extends SubsystemBase {
   NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
   NetworkTableEntry tx = table.getEntry("tx");
   NetworkTableEntry ty = table.getEntry("ty");
   NetworkTableEntry ta = table.getEntry("ta");
   NetworkTableEntry tv = table.getEntry("tv");
   NetworkTableEntry tid = table.getEntry("tid");
   NetworkTableEntry thor = table.getEntry("thor");
   NetworkTableEntry tvert = table.getEntry("tvert");

    // determining distance
  double targetOffsetAngle_Vertical = ty.getDouble(0.0);

  // how many degrees back is your limelight rotated from perfectly vertical?
  double limelightMountAngleDegrees = 0.0;

  // distance front the center of the Limelight lens to the floor
  double limelightLensHeightInches = 10.6;

  // distance from the target to the floor.
  double goalHeightInches = 53.25;

  double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
  double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

   double x = tx.getDouble(0.0);
     double y = ty.getDouble(0.0);
     double area = ta.getDouble(0.0);
     boolean validTarget = tv.getBoolean(true);
     double targetNumber = tid.getDouble(0);
     double horizontalLength = thor.getDouble(0.0);
     double verticalLength = tvert.getDouble(0.0);

  // calculate distance
  double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);

   /** Creates a new Limelight. */
   public LimeLight() {
    

   
   }

   @Override
   public void periodic() {
     // This method will be called once per scheduler run
      //read values periodically
     x = tx.getDouble(0.0);
     y = ty.getDouble(0.0);
     area = ta.getDouble(0.0);
     validTarget = tv.getBoolean(true);
     targetNumber = tid.getDouble(0);
     horizontalLength = thor.getDouble(0.0);
     verticalLength = tvert.getDouble(0.0);

     // adjust quotient as needed or if no time, just delete after &&
     if (targetNumber >= 0 && horizontalLength / verticalLength > 1.0) {
        validTarget = true;
     } else {
        validTarget = false;
     }

     //post to smart dashboard periodically
     SmartDashboard.putNumber("LimelightX", x);
     SmartDashboard.putNumber("LimelightY", y);
     SmartDashboard.putNumber("LimelightArea", area);
     SmartDashboard.putBoolean("ValidTarget", validTarget);
     SmartDashboard.putNumber("TargetNumber", targetNumber);
     SmartDashboard.putNumber("Distance", distanceFromLimelightToGoalInches);  
    
   }
 }