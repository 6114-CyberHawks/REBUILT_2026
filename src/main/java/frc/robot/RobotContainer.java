package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.IntakePivot;

public class RobotContainer {
    // Subsystem
    private final IntakePivot m_intakePivot = new IntakePivot(10, 11);  // Update CAN IDs
    
    // Xbox controller
    private final CommandXboxController m_controller = new CommandXboxController(0);
    
    public RobotContainer() {
        configureBindings();
    }
    
    private void configureBindings() {
        // A button - Deploy intake (PID controlled)
        m_controller.a()
            .onTrue(Commands.runOnce(() -> m_intakePivot.deployIntake(), m_intakePivot));
        
        // Y button - Stow intake (PID controlled)
        m_controller.y()
            .onTrue(Commands.runOnce(() -> m_intakePivot.stowIntake(), m_intakePivot));
    }
    
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // Example autonomous command: deploy intake, wait 2 seconds, stow intake
        return Commands.sequence(
            Commands.runOnce(() -> m_intakePivot.deployIntake(), m_intakePivot),
            Commands.waitSeconds(2.0),
            Commands.runOnce(() -> m_intakePivot.stowIntake(), m_intakePivot)
        );
    }
}
