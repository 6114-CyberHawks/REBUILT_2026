// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.SetClimb;
import frc.robot.commands.StopClimb;
import frc.robot.commands.ZeroClimb;
import frc.robot.subsystems.ClimberSubsystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ClimberSubsystem s_ClimberSubsystem = new ClimberSubsystem();

  private final SetClimb c_Climb;
  private final SetClimb c_UnClimb;
  private final StopClimb c_StopClimb;
  private final ZeroClimb c_ZeroClimb;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final XboxController m_driverController =
      new XboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    c_Climb = new SetClimb(s_ClimberSubsystem, 0);
    c_UnClimb = new SetClimb(s_ClimberSubsystem, 70);
    c_StopClimb = new StopClimb(s_ClimberSubsystem);
    c_ZeroClimb = new ZeroClimb(s_ClimberSubsystem);
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    new JoystickButton(m_driverController, XboxController.Button.kY.value)
        .onTrue(c_Climb);

    new JoystickButton(m_driverController, XboxController.Button.kA.value)
        .onTrue(c_UnClimb);

    new JoystickButton(m_driverController, XboxController.Button.kX.value)
        .onTrue(c_ZeroClimb);
      
    Trigger LiftUp = new Trigger(() -> m_driverController.getPOV() == 0);
    LiftUp.whileTrue(s_ClimberSubsystem.Up()).onFalse(c_StopClimb);

    Trigger LiftDown = new Trigger(() -> m_driverController.getPOV() == 180);
    LiftDown.whileTrue(s_ClimberSubsystem.Down()).onFalse(c_StopClimb);
    

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  //   // An example command will be run in autonomous
  //   return Autos.exampleAuto(m_exampleSubsystem);
  // }
}
