// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.commands.AutonomousLeft;
import frc.robot.commands.AutonomousRight;

public class AutonomousModeManager extends SubsystemBase {
  private AutonomousLeft c_AutonomousLeft;
  private AutonomousRight c_AutonomousRight;

  private Command c_Autonomous;

  /** Creates a new AutonomousModeManager. */
  public AutonomousModeManager(AutonomousLeft tc_AutonomousLeft, AutonomousRight tc_AutonomousRight) {
    c_AutonomousLeft = tc_AutonomousLeft;
    c_AutonomousRight = tc_AutonomousRight;

    c_Autonomous = null;
  }


  public Command putAutonomousMode(int Selection) {
    if (Selection == 1) {
      c_Autonomous = c_AutonomousLeft;
    } else if (Selection == 2) {
      c_Autonomous = c_AutonomousRight;
    } else {
      c_Autonomous = null;
    }

    return c_Autonomous;
  }
  /**
   * Will return: Left if left side autonomous mode is selected, right if right side autonomous, none if no mode is selected.
   * @return a string, all capitalized.
   */
  public String getAutonomousMode() {
    if (c_Autonomous == c_AutonomousLeft) {
      //System.out.println("Left Auto mode selected");
      return "Left";
    } else if (c_Autonomous == c_AutonomousRight) {
      //System.out.println("Right Auto mode selected");
      return "Right";
    } else {
      //System.out.println("No Auto mode selected");
      return "None";
    }
  }

  public boolean checkAutonomousDeselected() {
    if (c_Autonomous == null) {
      return true;
    } else {
      return false;
    }
  }

  public Command getAutonomousModeCommand() {
    return c_Autonomous;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
