
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

public class AutonomousRight extends Command {
  private final DriveSubsystem driveSubsystem;
  private final Timer timer;

  /** Creates a new AutonomousC. */
  public AutonomousRight(DriveSubsystem s_DriveSubsystem) {
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

    // Prints out Robot positions and rotation
    System.out.println("X axis(??) position of robot (estimated in inches): " +
    (driveSubsystem.getPose().getX()) + ", Y axis(??) position of robot (estimated in inches): " +
    (driveSubsystem.getPose().getY()) + ", Rotation of robot (estimated): " +
    driveSubsystem.getPose().getRotation().getDegrees() + ", Rotation: " +
    driveSubsystem.getHeading()
    );
    
    if (timer.get() < 1 && timer.get() > 0) {
      //driveSubsystem.StopAtAngle(0, 0.25);
      driveSubsystem.resetOdometry(Pose2d.kZero);
    }/*
    if (timer.get() < 4 && timer.get() > 1) {
      //driveSubsystem.StopAtPosition(0.0, Units.inchesToMeters(40), .1);
    }*/
    if (timer.get() < 2.5 && timer.get() > 1) {
      driveSubsystem.StopAtPosition(-Units.inchesToMeters(30), 0.0, .25);
    }
    if (timer.get() < 3.5 /*15*/ && timer.get() > 2.5 /*10*/) {
      System.out.println("Rotating to 30");

      driveSubsystem.StopAtAngle(30, 0.75);

      //angle = -30;
      //rot = 0.075;
      //driveSubsystem.StopAtAngle(angle, rot);
    }
    if (timer.get() < 11 /*20*/ && timer.get() > 10 /*15*/) {
      System.out.println("rotating to 180");

      driveSubsystem.StopAtAngle(180, 1);

      //angle = 0;
      //rot = 0.075;

      //driveSubsystem.StopAtAngle(angle, rot);

      //driveSubsystem.StopAtAngle(-60, 0.5); // The robot doesn't go back to 0...?
    }
    if (timer.get() < 13.5 && timer.get() > 11) {
      //driveSubsystem.resetOdometry(Pose2d.kZero);
      driveSubsystem.StopAtPosition(-Units.inchesToMeters(-20), 0.0, .25);
    }
    if (timer.get() < 14.95 && timer.get() > 13.5) {
      driveSubsystem.StopAtPosition(0.0, -Units.inchesToMeters(0.01), .1);
    }
    if (timer.get() > 15) {
      driveSubsystem.drive(0, 0, 0, false);
    }
  }
}