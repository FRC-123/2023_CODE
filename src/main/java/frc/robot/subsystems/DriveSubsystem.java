// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  // Constants
  private final static int SMART_CURRENT_LIMIT = 40;
  private final static double NOMINAL_DRIVE_VOLTAGE = 11.0;   // for voltage compensation

  private final CANSparkMax leftDrive = new CANSparkMax(DriveConstants.kLeftMotor1Port, MotorType.kBrushless);
  private final CANSparkMax leftDrivefollow = new CANSparkMax(DriveConstants.kLeftMotor2Port, MotorType.kBrushless);
  private final CANSparkMax rightDrive = new CANSparkMax(DriveConstants.kRightMotor1Port, MotorType.kBrushless);
  private final CANSparkMax rightDrivefollow = new CANSparkMax(DriveConstants.kRightMotor2Port, MotorType.kBrushless);

  private final SparkMaxPIDController leftController = leftDrive.getPIDController();
  private final SparkMaxPIDController rightController = rightDrive.getPIDController();
  // private DifferentialDriveOdometry odometry;
  // private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.098592, 4.222, 0.24104);
  // private final DifferentialDriveKinematics differentialDriveKinematics = new DifferentialDriveKinematics(0.6895);

  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(leftDrive, rightDrive);

  private final RelativeEncoder leftEncoder = leftDrive.getEncoder();
  private final RelativeEncoder rightEncoder = rightDrive.getEncoder();

  // The gyro sensor
  private final AHRS m_ahrs = new AHRS(edu.wpi.first.wpilibj.SPI.Port.kMXP);

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.

    // first, restore factory defaults on all motor controllers

    leftDrive.restoreFactoryDefaults();
    rightDrive.restoreFactoryDefaults();
    leftDrivefollow.restoreFactoryDefaults();
    rightDrivefollow.restoreFactoryDefaults();

    leftDrive.setInverted(false);
    rightDrive.setInverted(true);
    // do NOT invert the followers, they will mirror the output of the leader
    leftDrivefollow.setInverted(false);
    rightDrivefollow.setInverted(false);

    rightDrive.enableVoltageCompensation(NOMINAL_DRIVE_VOLTAGE);
    leftDrive.enableVoltageCompensation(NOMINAL_DRIVE_VOLTAGE);
    rightDrivefollow.enableVoltageCompensation(NOMINAL_DRIVE_VOLTAGE);
    leftDrivefollow.enableVoltageCompensation(NOMINAL_DRIVE_VOLTAGE);

    // reasonable value for testing
    leftDrive.setSmartCurrentLimit(SMART_CURRENT_LIMIT);
    rightDrive.setSmartCurrentLimit(SMART_CURRENT_LIMIT);
    leftDrivefollow.setSmartCurrentLimit(SMART_CURRENT_LIMIT);
    rightDrivefollow.setSmartCurrentLimit(SMART_CURRENT_LIMIT);

    leftDrive.setIdleMode(IdleMode.kCoast);
    rightDrive.setIdleMode(IdleMode.kCoast);
    leftDrivefollow.setIdleMode(IdleMode.kCoast);
    rightDrivefollow.setIdleMode(IdleMode.kCoast);

    rightDrivefollow.follow(rightDrive);
    leftDrivefollow.follow(leftDrive);
 

    leftEncoder.setPositionConversionFactor(DriveConstants.kPositionConvFactor);
    rightEncoder.setPositionConversionFactor(DriveConstants.kPositionConvFactor);
    leftEncoder.setVelocityConversionFactor(DriveConstants.kVelocityConvFactor);
    rightEncoder.setVelocityConversionFactor(DriveConstants.kVelocityConvFactor);

    leftController.setFeedbackDevice(leftEncoder);
    rightController.setFeedbackDevice(rightEncoder);
    
    leftController.setP(5.9245E-05, 0);
    rightController.setP(5.9245E-05, 0);
    leftController.setD(0, 0);
    rightController.setD(0, 0);
    leftController.setI(0, 0);
    rightController.setI(0, 0);
    leftController.setIZone(0, 0);
    rightController.setIZone(0, 0);
    leftController.setOutputRange(-1, 1, 0);
    rightController.setOutputRange(-1, 1, 0);
    leftController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
    rightController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);

    leftDrive.burnFlash();
    rightDrive.burnFlash();
    leftDrivefollow.burnFlash();
    rightDrivefollow.burnFlash();

    resetEncoders();
    m_odometry =
        new DifferentialDriveOdometry(
          m_ahrs.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
      m_ahrs.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity(), rightEncoder.getVelocity());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(
        m_ahrs.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition(), pose);
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftDrive.setVoltage(leftVolts);
    rightDrive.setVoltage(rightVolts);
    m_drive.feed();
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (leftEncoder.getPosition() + rightEncoder.getPosition()) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public RelativeEncoder getLeftEncoder() {
    return leftEncoder;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public RelativeEncoder getRightEncoder() {
    return rightEncoder;
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_ahrs.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_ahrs.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -m_ahrs.getRate();
  }
}
