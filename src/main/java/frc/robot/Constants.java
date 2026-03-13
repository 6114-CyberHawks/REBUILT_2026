// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class MotorFollowerConstants {
    public static final boolean TurretFollower1IsOppositeOrientationAsLeader = false;// is motor facing the same direction as leader
    public static final boolean TurretFollower2IsOppositeOrientationAsLeader = true; // is motor facing the same direction as leader
    public static final boolean IntakePivotFollowerIsOppositeOrientationAsLeader = true; // is motor facing the same direction as leader

  }

  public static class MotorIDs {
    public static final int kFrontLeftDrivingCanId = 7;
    public static final int kRearLeftDrivingCanId = 9;
    public static final int kFrontRightDrivingCanId = 4;
    public static final int kRearRightDrivingCanId = 2;

    public static final int kFrontLeftTurningCanId = 5;
    public static final int kRearLeftTurningCanId = 8;
    public static final int kFrontRightTurningCanId = 3;
    public static final int kRearRightTurningCanId = 1;

    public static final int HoodMotor = 11;
    public static final int TurretShootLeader = 12;
    public static final int TurretShootFollower1 = 13;
    public static final int TurretShootFollower2 = 14;
    public static final int FeedTurret = 15;
    public static final int ClimbMotor = 16;
    public static final int HopperMotor = 17;
    public static final int IntakePivotLeader = 18;
    public static final int IntakePivotFollower = 19;
    public static final int IntakeMotor = 20;
  }

  public static class DIOPortIDs {

    public static final int ClimbLimitSwitch = 0;

  }

  public final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;

    // Current Limits
    public static final int ShooterCurrentLimit = 60; // amps
    public static final int HoodMotorCurrentLimit = 20; // amps
    public static final int IntakePivotCurrentLimit = 40; // amps
    public static final int IntakeMotorCurrentLimit = 60; // amps
    public static final int HopperMotorCurrentLimit = 60; // amps
    public static final int ClimbMotorCurrentLimit = 60; // ampss

  }

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(23.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(23.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort1 = 2;
    public static final int kOperatorControllerPort2 = 1;

    public static final double kDriveDeadband = 0.05;
  }

  public static class ButtonBoxIDs{
     public static final int ClimbUp = 1; // 1SW1
     public static final int ClimbDown = 2; // 1SW2
     public static final int ClimbUpManual = 3; // 1SW3
     public static final int ClimbDownManual = 8; // 1SW8
     public static final int HoodClose = 4; // 1SW4
     public static final int HoodMid = 5; // 1SW5
     public static final int HoodFar = 6; // 1SW6
     public static final int HoodPass = 7; // 1SW7

     public static final int DeployIntake = 3; // 2SW2
     public static final int StowIntake = 2; // 2SW3
     public static final int IncreaseShoot = 4; // 2SW4
     public static final int DecreaseShoot = 5; // 2SW5
     public static final int RunIntake = 6; // 2SW6
     public static final int WiggleIntake = 7; // 2SW7
     public static final int ReverseIntake = 8; // 2SW8

  }
}
