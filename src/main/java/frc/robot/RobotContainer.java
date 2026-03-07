// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.math.MathUtil;
//import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.math.controller.ProfiledPIDController;
//import edu.wpi.first.math.filter.SlewRateLimiter;
//mport edu.wpi.first.math.geometry.Pose2d;
//import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.math.geometry.Translation2d;
//import edu.wpi.first.math.trajectory.Trajectory;
//import edu.wpi.first.math.trajectory.TrajectoryConfig;
//import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.PS4Controller.Button;
//import frc.robot.Constants.AutoConstants;
//import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.Autonomous;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
//import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
//import static edu.wpi.first.units.Units.Degrees;
//import java.util.List;

//import com.studica.frc.Navx;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Commands : naming convention is c_ for command
  private final Autonomous c_Autonomous;

  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  // Creates a SlewRateLimiter that limits the rate of change of the signal to 0.5 units per second
  //SlewRateLimiter filter = new SlewRateLimiter(0.01);

  double modifyAxis(double value, double exponent) {
      value = MathUtil.applyDeadband(value, 0.22 /* orginal: 0.12; Deadband */);

      return Math.copySign(Math.pow(Math.abs(value), exponent), value);
    }

    double modifyAxis(double value, double exponent, double deadband) {
      value = MathUtil.applyDeadband(value, deadband);

      return Math.copySign(Math.pow(Math.abs(value), exponent), value);
    }
  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    c_Autonomous = new Autonomous(m_robotDrive);
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
              /*
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband), // Drive
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband), // Strafe
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband), // Rotation
              */
                -modifyAxis(m_driverController.getLeftY(), 3.5, 0.075),
                -modifyAxis(m_driverController.getLeftX(), 3.5, 0.075),
                -modifyAxis(m_driverController.getRightX(), 4, 0.075),
              
                true),
            m_robotDrive));
  }
    //pid.calculate(, 1);


  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    /*new JoystickButton(m_driverController, Button.kR1.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
    */
    new JoystickButton(m_driverController, XboxController.Button.kStart.value)
        .onTrue(new InstantCommand(
            () -> m_robotDrive.zeroHeading(),
            m_robotDrive));

    new JoystickButton(m_driverController, XboxController.Button.kY.value)
        .onTrue(new InstantCommand(() -> System.out.println("Before Reset: " + 
          m_robotDrive.getHeading() + ", Yaw Reset(?); StatusCode: " +
          m_robotDrive.zeroHeading() + ", After Reset: " +
          m_robotDrive.getHeading()), /*Checked the values & reset Yaw*/
        m_robotDrive));
  }

  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    /* Default Automous Command
    
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
    */
    return c_Autonomous;
  }
}
