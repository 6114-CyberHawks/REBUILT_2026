
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.units.Units;
//import edu.wpi.first.math.util.Units;
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
// Default Auton, 1 coral + leave

    System.out.println(driveSubsystem.getPose().getX() + " " + driveSubsystem.getPose().getY() + " Raw Rot: " + DriveSubsystem.m_gyro.getYaw() + " Rot: " + DriveSubsystem.m_gyro.getYaw().in(Units.Degree));

    if (timer.get() < 3.5 && timer.get() > 0) {
      //driveSubsystem.StopAtPosition(Units.inchesToMeters(31), 0.0, .3, 0, 0);
    } if (timer.get() < 8.5 && timer.get() > 3.5) {
      driveSubsystem.StopAtAngle(180, 0.3);
    } if (timer.get() > 8.5) {
      driveSubsystem.drive(0, 0, 0, false);
    }
  }
}