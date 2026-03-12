// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ButtonBoxIDs;
import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.math.MathUtil;
// import frc.robot.commands.DecreaseFeedSpeed;
import frc.robot.commands.DecreaseShootSpeed;
import frc.robot.commands.DeployIntake;
import frc.robot.commands.FeedForward;
import frc.robot.commands.HopperForward;
import frc.robot.commands.HopperOcilate;
import frc.robot.commands.HopperReverse;
import frc.robot.commands.HopperStop;
// import frc.robot.commands.IncreaseFeedSpeed;
import frc.robot.commands.IncreaseShootSpeed;
import frc.robot.commands.ReverseFeed;
import frc.robot.commands.ReverseIntake;
import frc.robot.commands.ReverseShooter;
import frc.robot.commands.RunIntake;
import frc.robot.commands.SetClimb;
import frc.robot.commands.ShootTurret;
import frc.robot.commands.StopClimb;
import frc.robot.commands.StopFeed;
import frc.robot.commands.StopIntake;
import frc.robot.commands.StopIntakePivot;
import frc.robot.commands.StopShooter;
import frc.robot.commands.StowIntake;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutonomousLeft;
import frc.robot.commands.AutonomousRight;
import frc.robot.subsystems.AutonomousModeManager;
import frc.robot.subsystems.AlmostDataManager;
import frc.robot.subsystems.DriveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // subsystems
  private final TurretSubsystem s_TurretSubsystem = new TurretSubsystem();
  private final HopperSubsystem s_HopperSubsystem = new HopperSubsystem();
  private final HoodSubsystem s_HoodSubsystem = new HoodSubsystem();
  private final IntakeSubsystem s_IntakeSubsystem = new IntakeSubsystem();
  private final ClimberSubsystem s_ClimberSubsystem = new ClimberSubsystem();
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final AutonomousModeManager s_AutonomousModeManager;
  private final AlmostDataManager s_DataTableManager;

  // The robot's subsystems and commands are defined here...
  // shooter
  private final ShootTurret c_ShootTurret;
  private final StopShooter c_StopShooter;
  private final ReverseShooter c_ReverseShooter;
  private final FeedForward c_FeedForward;
  private final ReverseFeed c_ReverseFeed;
  private final StopFeed c_StopFeed;
  // private final IncreaseFeedSpeed c_IncreaseFeedSpeed;
  // private final DecreaseFeedSpeed c_DecreaseFeedSpeed;
  private final IncreaseShootSpeed c_IncreaseShootSpeed;
  private final DecreaseShootSpeed c_DecreaseShootSpeed;

  private final HopperForward c_hopperForward;
  private final HopperStop c_hopperStop;
  private final HopperReverse c_hopperReverse;
  private final HopperOcilate c_HopperOcilate;

  private final DeployIntake c_DeployIntake;
  private final StowIntake c_StowIntake;
  private final StopIntakePivot c_StopIntakePivot;
  private final RunIntake c_RunIntake;
  private final ReverseIntake c_ReverseIntake;
  private final StopIntake c_StopIntake;

  private final SetClimb c_Climb;
  private final SetClimb c_UnClimb;
  private final StopClimb c_StopClimb;

  private final AutonomousLeft c_AutonomousLeft;
  private final AutonomousRight c_AutonomousRight;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final XboxController m_driverController = new XboxController(OperatorConstants.kDriverControllerPort);
  private final CommandJoystick m_buttonBox1 = new CommandJoystick(OperatorConstants.kOperatorControllerPort1);
  private final CommandJoystick m_buttonBox2 = new CommandJoystick(OperatorConstants.kOperatorControllerPort2);

  // Creates a SlewRateLimiter that limits the rate of change of the signal to 0.5
  // units per second
  // SlewRateLimiter filter = new SlewRateLimiter(0.01);
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
    s_DataTableManager = new AlmostDataManager(null); // DO NOT USE NORMALLY!!

    // shooter
    c_ShootTurret = new ShootTurret(s_TurretSubsystem);
    c_StopShooter = new StopShooter(s_TurretSubsystem);
    c_ReverseShooter = new ReverseShooter(s_TurretSubsystem);
    c_IncreaseShootSpeed = new IncreaseShootSpeed(s_TurretSubsystem);
    c_DecreaseShootSpeed = new DecreaseShootSpeed(s_TurretSubsystem);

    // feeder
    c_FeedForward = new FeedForward(s_TurretSubsystem);
    c_ReverseFeed = new ReverseFeed(s_TurretSubsystem);
    c_StopFeed = new StopFeed(s_TurretSubsystem);
    // c_IncreaseFeedSpeed = new IncreaseFeedSpeed(s_TurretSubsystem);
    // c_DecreaseFeedSpeed = new DecreaseFeedSpeed(s_TurretSubsystem);

    // hood

    // hopper
    c_hopperForward = new HopperForward(s_HopperSubsystem);
    c_hopperStop = new HopperStop(s_HopperSubsystem);
    c_hopperReverse = new HopperReverse(s_HopperSubsystem);
    c_HopperOcilate = new HopperOcilate(s_HopperSubsystem);

    // intake
    c_DeployIntake = new DeployIntake(s_IntakeSubsystem);
    c_StowIntake = new StowIntake(s_IntakeSubsystem);
    c_StopIntakePivot = new StopIntakePivot(s_IntakeSubsystem);
    c_RunIntake = new RunIntake(s_IntakeSubsystem);
    c_ReverseIntake = new ReverseIntake(s_IntakeSubsystem);
    c_StopIntake = new StopIntake(s_IntakeSubsystem);

    // climb
    c_Climb = new SetClimb(s_ClimberSubsystem, 0);
    c_UnClimb = new SetClimb(s_ClimberSubsystem, 70);
    c_StopClimb = new StopClimb(s_ClimberSubsystem);

    // autos
    c_AutonomousLeft = new AutonomousLeft(m_robotDrive, s_DataTableManager);
    c_AutonomousRight = new AutonomousRight(m_robotDrive, s_DataTableManager);

    s_AutonomousModeManager = new AutonomousModeManager(c_AutonomousLeft, c_AutonomousRight);

    s_DataTableManager.setAutonomousModeManager(s_AutonomousModeManager);

    // initilize commands
    // Configure the trigger bindings
    configureBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                /*
                 * -MathUtil.applyDeadband(m_driverController.getLeftY(),
                 * OIConstants.kDriveDeadband), // Drive
                 * -MathUtil.applyDeadband(m_driverController.getLeftX(),
                 * OIConstants.kDriveDeadband), // Strafe
                 * -MathUtil.applyDeadband(m_driverController.getRightX(),
                 * OIConstants.kDriveDeadband), // Rotation
                 */
                -modifyAxis(m_driverController.getLeftY(), 3.5, 0.075),
                -modifyAxis(m_driverController.getLeftX(), 3.5, 0.075),
                -modifyAxis(m_driverController.getRightX(), 4, 0.075),

                true),
            m_robotDrive));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    new JoystickButton(m_driverController, XboxController.Button.kStart.value)
        .onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));

    // turret / hopper
    new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
        .whileTrue(c_ShootTurret).whileTrue(c_hopperForward).whileTrue(c_FeedForward) // held
        .onFalse(c_StopShooter).whileFalse(c_HopperOcilate).onFalse(c_StopFeed); // released

    new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
        .whileTrue(c_ReverseShooter).whileTrue(c_hopperReverse).whileTrue(c_ReverseFeed) // held
        .onFalse(c_StopShooter).onFalse(c_StopFeed).whileFalse(c_HopperOcilate); // released

    // Hood
    new JoystickButton(m_driverController, XboxController.Button.kY.value)
        .onTrue(new InstantCommand(s_HoodSubsystem::incrementPosition, s_HoodSubsystem));

    new JoystickButton(m_driverController, XboxController.Button.kA.value)
        .onTrue(new InstantCommand(s_HoodSubsystem::decrementPosition, s_HoodSubsystem));

    m_buttonBox1.button(ButtonBoxIDs.HoodClose)
        .onTrue(new InstantCommand(s_HoodSubsystem::setLowPosition, s_HoodSubsystem));
    m_buttonBox1.button(ButtonBoxIDs.HoodMid)
        .onTrue(new InstantCommand(s_HoodSubsystem::setMidPosition, s_HoodSubsystem));
    m_buttonBox1.button(ButtonBoxIDs.HoodFar)
        .onTrue(new InstantCommand(s_HoodSubsystem::setHighPosition, s_HoodSubsystem));
    m_buttonBox1.button(ButtonBoxIDs.HoodPass)
        .onTrue(new InstantCommand(s_HoodSubsystem::setPassPosition, s_HoodSubsystem));

    // Intake
    m_buttonBox2.button(ButtonBoxIDs.DeployIntake)
        .whileTrue(c_DeployIntake).onFalse(c_StopIntakePivot);

    m_buttonBox2.button(ButtonBoxIDs.StowIntake)
        .whileTrue(c_StowIntake).onFalse(c_StopIntakePivot);

    m_buttonBox2.button(ButtonBoxIDs.RunIntake)
        .whileTrue(c_RunIntake).onFalse(c_StopIntake);

    m_buttonBox2.button(ButtonBoxIDs.ReverseIntake)
        .whileTrue(c_ReverseIntake).onFalse(c_StopIntake);

    // climb
    m_buttonBox1.button(ButtonBoxIDs.ClimbDown)
        .onTrue(c_Climb);
      
    m_buttonBox1.button(ButtonBoxIDs.ClimbUp)
        .onTrue(c_UnClimb);
    
    m_buttonBox1.button(ButtonBoxIDs.ClimbDownManual)
        .whileTrue(s_ClimberSubsystem.Down()).onFalse(c_StopClimb);

    m_buttonBox1.button(ButtonBoxIDs.ClimbUpManual)
        .whileTrue(s_ClimberSubsystem.Up()).onFalse(c_StopClimb);

    // button box shooter
    m_buttonBox2.button(ButtonBoxIDs.IncreaseShoot)
        .onTrue(c_IncreaseShootSpeed);

    m_buttonBox2.button(ButtonBoxIDs.DecreaseShoot)
        .onTrue(c_DecreaseShootSpeed);

    // LiftDown.whileTrue(s_ClimberSubsystem.Down()).onFalse(c_StopClimb);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    /*
     * Default Automous Command
     * 
     * // Create config for trajectory
     * TrajectoryConfig config = new TrajectoryConfig(
     * AutoConstants.kMaxSpeedMetersPerSecond,
     * AutoConstants.kMaxAccelerationMetersPerSecondSquared)
     * // Add kinematics to ensure max speed is actually obeyed
     * .setKinematics(DriveConstants.kDriveKinematics);
     * 
     * // An example trajectory to follow. All units in meters.
     * Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
     * // Start at the origin facing the +X direction
     * new Pose2d(0, 0, new Rotation2d(0)),
     * // Pass through these two interior waypoints, making an 's' curve path
     * List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
     * // End 3 meters straight ahead of where we started, facing forward
     * new Pose2d(3, 0, new Rotation2d(0)),
     * config);
     * 
     * var thetaController = new ProfiledPIDController(
     * AutoConstants.kPThetaController, 0, 0,
     * AutoConstants.kThetaControllerConstraints);
     * thetaController.enableContinuousInput(-Math.PI, Math.PI);
     * 
     * SwerveControllerCommand swerveControllerCommand = new
     * SwerveControllerCommand(
     * exampleTrajectory,
     * m_robotDrive::getPose, // Functional interface to feed supplier
     * DriveConstants.kDriveKinematics,
     * 
     * // Position controllers
     * new PIDController(AutoConstants.kPXController, 0, 0),
     * new PIDController(AutoConstants.kPYController, 0, 0),
     * thetaController,
     * m_robotDrive::setModuleStates,
     * m_robotDrive);
     * 
     * // Reset odometry to the starting pose of the trajectory.
     * m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());
     * 
     * // Run path following command, then stop at the end.
     * return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0,
     * false));
     */
    if (s_AutonomousModeManager.getAutonomousMode() == "None") {
      System.out.println("\n\n\nWARNING: NO AUTO SELECTED! Please select an autonomous mode!\n\n\n");
    }

    return s_AutonomousModeManager.getAutonomousModeCommand();
  }

}
