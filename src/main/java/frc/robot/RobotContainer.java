// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj.XboxController.Button;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.DefaultDriveCommand;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HiArmSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.LoArmSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final HiArmSubsystem m_hiArm = new HiArmSubsystem();
  private final LoArmSubsystem m_loArm = new LoArmSubsystem();


  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_armController = new XboxController(OIConstants.kArmControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    m_robotDrive.setDefaultCommand(new DefaultDriveCommand(m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // Drive at half speed when the right bumper is held
    new JoystickButton(m_armController, Button.kA.value)
        .onTrue(new InstantCommand(() -> m_loArm.moveToPosition(0)));
    new JoystickButton(m_armController, Button.kY.value)
        .onTrue(new InstantCommand(() -> m_loArm.moveToPosition(90)));
    new JoystickButton(m_armController, Button.kX.value)
        .onTrue(new InstantCommand(() -> m_hiArm.moveToPosition(60)));
    new JoystickButton(m_driverController, Button.kLeftBumper.value)
        .onTrue(new InstantCommand(() -> LedSubsystem.toggle_cube()));
    new JoystickButton(m_driverController, Button.kRightBumper.value)
        .onTrue(new InstantCommand(() -> LedSubsystem.toggle_cone()));
    new JoystickButton(m_armController, Button.kRightBumper.value)
        .onTrue(new InstantCommand(() -> m_hiArm.intakeCone()))
        .onFalse(new InstantCommand(() -> m_hiArm.stopRollers()));
    new JoystickButton(m_armController, Button.kLeftBumper.value)
        .onTrue(new InstantCommand(() ->  m_loArm.moveRollers(-0.6)))
        .onFalse(new InstantCommand(() -> m_loArm.stopRollers()));
    // Led bar triggers
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                DriveConstants.ksVolts,
                DriveConstants.kvVoltSecondsPerMeter,
                DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics,
            5);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1.5, 1.5), new Translation2d(3, 0), new Translation2d(1.5, -1.5)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(0, 0, new Rotation2d(90)),
            // Pass config
            config);

    /*RamseteCommand ramseteCommand =
        new RamseteCommand(
            exampleTrajectory,
            m_robotDrive::getPose,
            new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
            new SimpleMotorFeedforward(
                DriveConstants.ksVolts,
                DriveConstants.kvVoltSecondsPerMeter,
                DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics,
            m_robotDrive::getWheelSpeeds,
            new PIDController(DriveConstants.kPDriveVel, 0, 0),
            new PIDController(DriveConstants.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            m_robotDrive::tankDriveVolts,
            m_robotDrive);*/
        RamseteCommand ramseteCommand = 
            new RamseteCommand(exampleTrajectory, m_robotDrive::getPose, new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta), DriveConstants.kDriveKinematics, m_robotDrive::tankMetersPerSecond, m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));
  }

    /**
     * @param value
     * @param deadband
     * @param maxMagnitude
     * @return
     */
    private static double modifyAxis(double value, double deadband, double maxMagnitude) {
        
        // validate input
        double joyout = MathUtil.clamp(Math.abs(value),0.0,1.0);
        // apply deadband, scale remaining range 0..maxMagnitude
        joyout = MathUtil.applyDeadband(joyout, deadband, maxMagnitude);
        // square input, restore sign
        joyout = Math.copySign(joyout*joyout, value);

        return joyout;
    }

}

