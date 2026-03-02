// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.IntakePivot;  // Import your subsystem

public class RobotContainer {
    // Create the subsystem
    private final IntakePivot m_intakePivot = new IntakePivot(10, 11);
    
    // Button board on USB port 1
    private final Joystick m_buttonBoard = new Joystick(1);
    
    // Buttons
    private final JoystickButton m_deployButton = new JoystickButton(m_buttonBoard, 1);
    private final JoystickButton m_stowButton = new JoystickButton(m_buttonBoard, 2);
    
    public RobotContainer() {
        configureBindings();
    }
    
    private void configureBindings() {
    // Deploy button with debug output
    m_deployButton
        .whileTrue(Commands.run(() -> {
            System.out.println("Button 1 pressed - deploying!");
            m_intakePivot.deployIntake();
        }, m_intakePivot))
        .onFalse(Commands.runOnce(() -> {
            System.out.println("Button 1 released - stopping!");
            m_intakePivot.stop();
        }, m_intakePivot));
    
    // Stow button with debug output
    m_stowButton
        .whileTrue(Commands.run(() -> {
            System.out.println("Button 2 pressed - stowing!");
            m_intakePivot.stowIntake();
        }, m_intakePivot))
        .onFalse(Commands.runOnce(() -> {
            System.out.println("Button 2 released - stopping!");
            m_intakePivot.stop();
        }, m_intakePivot));
}

    
    public IntakePivot getIntakePivot() {
        return m_intakePivot;
    }
}

