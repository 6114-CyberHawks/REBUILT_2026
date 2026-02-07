// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.FieldLocationHelpers;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
//import constants
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.MotorIDs;

public class DriveSubsystem extends SubsystemBase {

  public static Spark LeftMotorFront;
  public static Spark LeftMotorRear;
  public static Spark RightMotorFront;
  public static Spark RightMotorRear;

  DifferentialDrive drive;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    LeftMotorFront = new Spark(MotorIDs.LeftMotorFront);
    LeftMotorFront.setInverted(false);
    LeftMotorRear = new Spark(MotorIDs.LeftMotorRear);
    LeftMotorRear.setInverted(false);
    LeftMotorRear.addFollower(LeftMotorFront);

    RightMotorFront = new Spark(MotorIDs.RigthMotorFront);
    RightMotorFront.setInverted(false);
    RightMotorRear = new Spark(MotorIDs.RigthMotorRear);
    RightMotorRear.setInverted(false);
    RightMotorRear.addFollower(RightMotorFront);

    drive = new DifferentialDrive(LeftMotorRear, RightMotorRear);
  }

  public void RotateToTarget() { // keep this function
    int[] validIDs = { 1 };
    LimelightHelpers.SetFiducialIDFiltersOverride("kitbot", validIDs);
    double KpAim = -0.1f;
    double KpDistance = -0.1f;
    double min_aim_command = 0.05f;

    // Get target coordinates from Limelight
    double tx = LimelightHelpers.getTX("kitbot");
    double ty = LimelightHelpers.getTY("kitbot");

    // Check if we should use the joystick input
    if (RobotContainer.m_driverController.getRawAxis(0) == 0) { 
      double heading_error = -tx; // Error in heading
      double distance_error = -ty; // Error in distance
      double steering_adjust = 0.0f;
      System.out.println("tx: " + tx);
      // Calculate steering adjustment based on heading error
      if (tx > 1.0) {
        steering_adjust = KpAim * heading_error - min_aim_command;
      } else if (tx < -1.0) {
        steering_adjust = KpAim * heading_error + min_aim_command;
      }

      double distance_adjust = KpDistance * distance_error;

      // Calculate final motor speeds
      double left_command = steering_adjust + distance_adjust; // Adjust left track
      double right_command = -steering_adjust + distance_adjust; // Adjust right track

      // Set the motor speeds using DifferentialDrive
      drive.tankDrive(left_command, right_command);
    }
  }

  public void FindDistanceToTarget() { // keep this function
    double a = 0.00000016516150;
    System.out.println(a);

    int[] validIDs = { 9, 10, 11, 2 };
    LimelightHelpers.SetFiducialIDFiltersOverride("kitbot", validIDs);
    // double[] targetVars = LimelightHelpers.getBotPose_TargetSpace("Kitbot"); //
    // targetVars contains variables: [X, Y, Z, Yaw, Pitch, Roll]
    double[] h = LimelightHelpers.getBotPose("kitbot");
    if (h[6] >= 2)
      a = FieldLocationHelpers.getDistanceToTagMeters("kitbot",
          (int) LimelightHelpers.getFiducialID("Kitbot"), true);

    System.out.println(a);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void driveWithJoysticks(XboxController controller, double speed) {
    drive.arcadeDrive(controller.getRawAxis(OperatorConstants.XboxLeft_Y_Axis) * speed,
        controller.getRawAxis(OperatorConstants.XboxLeft_X_Axis) * speed);
  }

  public void driveForward(double speed) {
    drive.tankDrive(speed, speed);
  }

  public void stop() {
    drive.stopMotor();
  }
}
