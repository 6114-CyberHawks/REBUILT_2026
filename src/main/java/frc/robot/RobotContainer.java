package frc.robot;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.IntakePivot;

public class RobotContainer {
    // Subsystem
    private final IntakePivot m_intakePivot = new IntakePivot(16, 11);
    
    // Xbox controller on port 0
    private final CommandXboxController m_controller = new CommandXboxController(0);
    
    public RobotContainer() {
        configureBindings();
    }
    
    private void configureBindings() {
        // A button - Deploy intake (down)
        m_controller.a()
            .whileTrue(Commands.run(() -> {
                System.out.println("A button pressed - deploying!");
                m_intakePivot.deployIntake();
            }, m_intakePivot))
            .onFalse(Commands.runOnce(() -> {
                System.out.println("A button released - stopping!");
                m_intakePivot.stop();
            }, m_intakePivot));
        
        // Y button - Stow intake (up)
        m_controller.y()
            .whileTrue(Commands.run(() -> {
                System.out.println("Y button pressed - stowing!");
                m_intakePivot.stowIntake();
            }, m_intakePivot))
            .onFalse(Commands.runOnce(() -> {
                System.out.println("Y button released - stopping!");
                m_intakePivot.stop();
            }, m_intakePivot));
    }
}