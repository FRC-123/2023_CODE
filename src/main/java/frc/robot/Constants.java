// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    public static final int kLeftMotor1Port = 1;
    public static final int kLeftMotor2Port = 2;
    public static final int kRightMotor1Port = 7;
    public static final int kRightMotor2Port = 4;

    public static final boolean kLeftMotorReversed = false;
    public static final boolean kRightMotorReversed = true;

    public static final double kTrackwidthMeters = 0.6895;
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);

    public static final int kEncoderCPR = 4096;
    public static final double kWheelDiameterMeters = 0.1524;
    public static final double kGearRatio = 16.37;
    public static final double kPositionConvFactor = ( Math.PI * kWheelDiameterMeters ) / kGearRatio;
    public static final double kVelocityConvFactor = kPositionConvFactor / 60.0;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
    // values for your robot.
    public static final double ksVolts = 0.098592;
    public static final double kvVoltSecondsPerMeter = 4.222;
    public static final double kaVoltSecondsSquaredPerMeter = 0.24104;

    // Example value only - as above, this must be tuned for your drive!
    public static final double kPDriveVel = 5.9245E-05; // sparkmax internal
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class TeleopDriveConstants {
    public static final double DEADBAND = 0.1;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 0.5;
    public static final double kMaxAccelerationMetersPerSecondSquared = 0.1;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
  }
}
