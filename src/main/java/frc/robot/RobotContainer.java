package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.IntakePivot;

public class RobotContainer {
    // Subsystem - UPDATE THESE CAN IDs TO MATCH YOUR ROBOT
    private final IntakePivot m_intakePivot = new IntakePivot(10, 11);
    
    // Xbox controller
    private final CommandXboxController m_controller = new CommandXboxController(0);
    
    public RobotContainer() {
        configureBindings();
    }
    
    private void configureBindings() {
        // A button - Deploy intake (hold to move down)
        m_controller.a()
            .whileTrue(Commands.run(() -> m_intakePivot.deploy(), m_intakePivot))
            .onFalse(Commands.runOnce(() -> m_intakePivot.stop(), m_intakePivot));
        
        // Y button - Stow intake (hold to move up)
        m_controller.y()
            .whileTrue(Commands.run(() -> m_intakePivot.stow(), m_intakePivot))
            .onFalse(Commands.runOnce(() -> m_intakePivot.stop(), m_intakePivot));
        
        // B button - Emergency stop
        m_controller.b()
            .onTrue(Commands.runOnce(() -> m_intakePivot.stop(), m_intakePivot));
    }
    
    public Command getAutonomousCommand() {
        // Simple autonomous: deploy for 2 seconds, then stow for 2 seconds
        return Commands.sequence(
            Commands.runOnce(() -> m_intakePivot.deploy(), m_intakePivot),
            Commands.waitSeconds(2.0),
            Commands.runOnce(() -> m_intakePivot.stop(), m_intakePivot),
            Commands.waitSeconds(0.5),
            Commands.runOnce(() -> m_intakePivot.stow(), m_intakePivot),
            Commands.waitSeconds(2.0),
            Commands.runOnce(() -> m_intakePivot.stop(), m_intakePivot)
        );
    }
}
