// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    public static final boolean Follower1IsOppositeOrientationAsLeader = false; // is motor facing the same direction as leader
    public static final boolean Follower2IsOppositeOrientationAsLeader = true; // is motor facing the same direction as the leader
  }
  
  public static class MotorIDs {
    public static final int TurretShootLeader = 12;
    public static final int TurretShootFollower1 = 14;
    public static final int TurretShootFollower2 = 19;
    public static final int FeedTurret = 7;
    public static final int TurretHood = 11;
    public static final int Climber = 14;
  }

  public static class DIOPortIDs {

    public static final int HoodBottomLimit = 0;
    public static final int ClimbBeamBreakLimit = 0;
    public static final int ClimbLimitSwitch = 0;

  }

  public final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;

    public static final int bigMotorCurrentLimit = 60; // amps
    public static final int smallMotorCurrentLimit = 20; // amps
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
