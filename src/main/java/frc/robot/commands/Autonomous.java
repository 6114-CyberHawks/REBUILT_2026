
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
/*
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.math.MathUtil;
*/
import edu.wpi.first.math.util.Units;

public class Autonomous extends Command {
  private final DriveSubsystem driveSubsystem;
  private final Timer timer;

  /** Creates a new AutonomousC. */
  public Autonomous(DriveSubsystem s_DriveSubsystem) {
    timer = new Timer();
    driveSubsystem = s_DriveSubsystem;

    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    driveSubsystem.resetEncoders();
    driveSubsystem.resetOdometry(driveSubsystem.getPose());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Default Auton

    // Prints out Robot positions and rotation, has been moved to Robot.java
    System.out.println("X axis(??) position of robot (estimated in inches): " +
    (driveSubsystem.getPose().getX()) + ", Y axis(??) position of robot (estimated in inches): " +
    (driveSubsystem.getPose().getY()) + ", Rotation of robot (estimated): " +
    driveSubsystem.getPose().getRotation().getDegrees() + ", Raw Rotation: " +
    DriveSubsystem.m_gyro.getYaw().in(Degrees) + ", Rotation: " +
    Math.round(DriveSubsystem.m_gyro.getYaw().in(Degrees))
    );
    
    if (timer.get() < 3.5 && timer.get() > 0) {
      //driveSubsystem.StopAtAngle(0, 0.25);
      driveSubsystem.resetOdometry(Pose2d.kZero);
    }
    if (timer.get() < 7 && timer.get() > 3.5) {
      driveSubsystem.StopAtPosition(driveSubsystem.getPose().getX(), Units.inchesToMeters(40), .1, 0, 0);
    }
      if (timer.get() < 16 && timer.get() > 7) {
      driveSubsystem.StopAtPosition(-Units.inchesToMeters(98), driveSubsystem.getPose().getY(), .1, 0, 0);
    }
    if (timer.get() < 20 && timer.get() > 17) {
      driveSubsystem.StopAtPosition(driveSubsystem.getPose().getX(), -Units.inchesToMeters(0.01), .05, 0, 0);

    }
    if (timer.get() > 20) {
      driveSubsystem.drive(0, 0, 0, false);
    }
  }
}