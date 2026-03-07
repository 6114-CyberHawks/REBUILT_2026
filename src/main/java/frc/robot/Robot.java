// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.BooleanArrayEntry;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.util.Color;
//import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DataTableManager;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.SensorManager;

//import edu.wpi.first.wpilibj.XboxController;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  //public BooleanPublisher IRSensor;

  // LED variables and constants
  private AddressableLED FrontLEDBar;
  //private AddressableLED BackLEDBar;
  private AddressableLEDBuffer LEDBarBuffer;
  private final LEDPattern offLEDs = LEDPattern.kOff;
  //private final LEDPattern greenLEDs = LEDPattern.solid(Color.kGreen);
  //private final LEDPattern purpleLEDs = LEDPattern.solid(Color.kMidnightBlue);
  //private final LEDPattern redLEDs = LEDPattern.solid(Color.kRed);
  private final LEDPattern rainbowLEDs = LEDPattern.rainbow(255, 128);
  private final LEDPattern orangeLEDs = LEDPattern.solid(Color.kOrange);
  // Our LED strip has a density of 12 LEDs per meter
  private static final Distance kLedSpacing = Units.Meters.of(1 / 12.0);
  private final LEDPattern scrollingRainbowLEDs = rainbowLEDs.scrollAtAbsoluteSpeed(Units.MetersPerSecond.of(1), kLedSpacing);
  
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  //private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    //IRSensor = NetworkTableInstance.getDefault().getTable("datatable").getBooleanTopic("IR Sensor").publish();

    // Setup LED Bars
    LEDBarBuffer = new AddressableLEDBuffer(15); // Number of LEDs we want to control

    FrontLEDBar = new AddressableLED(1); // LED bar connected to PWM port 0 on RoboRio
    FrontLEDBar.setLength(LEDBarBuffer.getLength());
    FrontLEDBar.setData(LEDBarBuffer);
    FrontLEDBar.start();

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    //System.out.print("");
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    offLEDs.applyTo(LEDBarBuffer);
    FrontLEDBar.setData(LEDBarBuffer);
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    orangeLEDs.applyTo(LEDBarBuffer);
    FrontLEDBar.setData(LEDBarBuffer);

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    /*
    if ((m_driverController.getLeftY() >= 0.9) || (m_driverController.getLeftX() <= -0.9)) {m_driverController.setRumble(RumbleType.kLeftRumble, 0.5);} else {m_driverController.setRumble(RumbleType.kLeftRumble, 0.0);}
    if ((m_driverController.getLeftX() >= 0.9) || (m_driverController.getLeftX() <= -0.9)) {m_driverController.setRumble(RumbleType.kLeftRumble, 0.5);} else {m_driverController.setRumble(RumbleType.kLeftRumble, 0.0);}
    if ((m_driverController.getRightX() >= 0.9) || (m_driverController.getRightX() <= -0.9)) {m_driverController.setRumble(RumbleType.kRightRumble, 0.5);} else {m_driverController.setRumble(RumbleType.kRightRumble, 0.0);}
    */
    
    System.out.printf("LeftY: %f; LeftX: %f; RightX: %f", m_driverController.getLeftY(), m_driverController.getLeftX(), m_driverController.getRightX());
    
    //System.out.println(m_gryo.getYaw());
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();

    System.out.println("IR Sensor: " + new SensorManager().getIRSenor());

    scrollingRainbowLEDs.applyTo(LEDBarBuffer);
    //FrontLEDBar.setData(LEDBarBuffer);
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    /*
    boolean Sensor = new Testing().getIRSenor();
    IRSensor.set(Sensor);
    System.out.println("Sensor(?): " + Sensor);
    */
  }
}
