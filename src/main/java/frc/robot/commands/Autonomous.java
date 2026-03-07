
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

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
    int angle;
    double rot;
    // Prints out Robot positions and rotation
    System.out.println("X axis(??) position of robot (estimated in inches): " +
    (driveSubsystem.getPose().getX()) + ", Y axis(??) position of robot (estimated in inches): " +
    (driveSubsystem.getPose().getY()) + ", Rotation of robot (estimated): " +
    driveSubsystem.getPose().getRotation().getDegrees() + ", Raw Rotation: " +
    DriveSubsystem.m_gyro.getYaw().getValue().in(Degrees) + ", Rotation: " +
    Math.round(DriveSubsystem.m_gyro.getYaw().getValue().in(Degrees))
    );
    
    if (timer.get() < 1 && timer.get() > 0) {
      //driveSubsystem.StopAtAngle(0, 0.25);
      driveSubsystem.resetOdometry(Pose2d.kZero);
    }
    if (timer.get() < 4 && timer.get() > 1) {
      //driveSubsystem.StopAtPosition(0.0, Units.inchesToMeters(40), .1);
    }
    if (timer.get() < 10 && timer.get() > 4) {
      driveSubsystem.StopAtPosition(-Units.inchesToMeters(40), 0.0, .1);
    }
    if (timer.get() < 15 /*15*/ && timer.get() > 10 /*10*/) {
      System.out.println("Rotating to -30");

      driveSubsystem.StopAtAngle(-30, 0.5);

      //angle = -30;
      //rot = 0.075;
      //driveSubsystem.StopAtAngle(angle, rot);
    }
    if (timer.get() < 20 /*20*/ && timer.get() > 15 /*15*/) {
      System.out.println("Reseting rotation to 0");

      driveSubsystem.StopAtAngle(0, 0.5);

      //angle = 0;
      //rot = 0.075;

      //driveSubsystem.StopAtAngle(angle, rot);

      //driveSubsystem.StopAtAngle(-60, 0.5); // The robot doesn't go back to 0...?
    }
    if (timer.get() < 26 && timer.get() > 20) {
      driveSubsystem.StopAtPosition(-Units.inchesToMeters(98), 0.0, .1);
    }
    if (timer.get() < 28.5 && timer.get() > 26) {
      driveSubsystem.StopAtPosition(0.0, -Units.inchesToMeters(0.01), .05);
    }
    if (timer.get() > 30) {
      driveSubsystem.drive(0, 0, 0, false);
    }
  }
}