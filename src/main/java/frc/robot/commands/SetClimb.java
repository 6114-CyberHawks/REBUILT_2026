// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetClimb extends Command {
  private final ClimberSubsystem climbSubsystem;
  private final double TopSpeed = .8;

  @SuppressWarnings("unused") private boolean Lifting;
  @SuppressWarnings("unused") private boolean Lowering;
  int desiredPosition;
  public Boolean targetMet;
  double positionTolerance = 2; // Can't be 0! May need to change based on speed
  double currentPosition;

  /** Creates a new SetClimb. */
  public SetClimb(ClimberSubsystem s_ClimberSubsystem, int InputPosition) {
    climbSubsystem = s_ClimberSubsystem;
    desiredPosition = InputPosition;
    addRequirements(climbSubsystem);
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Lifting = false;
    Lowering = false;
    currentPosition = climbSubsystem.GetEncoderPosition();
    if(currentPosition < (desiredPosition - positionTolerance)) {
      Lifting = true;
    } else if (currentPosition > (desiredPosition + positionTolerance)) {
      Lowering = true;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentPosition = climbSubsystem.GetEncoderPosition();
    if(currentPosition < (desiredPosition - positionTolerance)) {
      climbSubsystem.SetSpeed(TopSpeed);
      System.out.println("Lower than point called for");
    } else if (currentPosition > (desiredPosition- positionTolerance)) {
      climbSubsystem.SetSpeed(-TopSpeed);
      System.out.println("Higher than point called for");
    }

    targetMet = (currentPosition <= (desiredPosition + positionTolerance)) && (currentPosition >= (desiredPosition - positionTolerance));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climbSubsystem.Stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return targetMet;
  }
}
