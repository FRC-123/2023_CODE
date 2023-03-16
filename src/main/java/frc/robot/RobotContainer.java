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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
  private final LoArmSubsystem m_loArm = new LoArmSubsystem(m_hiArm);


  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_armController = new XboxController(OIConstants.kArmControllerPort);
  CommandXboxController m_armControllerCommand = new CommandXboxController(OIConstants.kArmControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    m_robotDrive.setDefaultCommand(new DefaultDriveCommand(m_robotDrive));

    SendableChooser<AutoType> autoType = new SendableChooser<AutoType>();
    autoType.addOption("Normal", AutoType.Normal);
    autoType.addOption("Balence", AutoType.Balence);
    autoType.setDefaultOption("Normal", AutoType.Normal);
    SendableChooser<AutoPiece> autoPiece = new SendableChooser<AutoPiece>();
    autoPiece.addOption("Cone", AutoPiece.Cone);
    autoPiece.addOption("Cube", AutoPiece.Cube);
    autoPiece.setDefaultOption("Cube", AutoPiece.Cube);
    SmartDashboard.putNumber("Auto Distance", 4.1146);
    SmartDashboard.putData("Auto Type", autoType);
    SmartDashboard.putData("Auto Piece", autoPiece);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // Drive at half speed when the right bumper is held
    new Trigger(this::L1Up)
        .onTrue(new InstantCommand(() -> m_loArm.moveToPosition(0)));
    new Trigger(this::L1Down)
        .onTrue(new InstantCommand(() -> m_loArm.moveToPosition(90)));
    
    m_armControllerCommand.povUp().onTrue(new InstantCommand(m_hiArm::incrementArm, m_hiArm));
    m_armControllerCommand.povDown().onTrue(new InstantCommand(m_hiArm::decrementArm, m_hiArm));

    new JoystickButton(m_armController, Button.kRightBumper.value)
        .onTrue(new InstantCommand(() -> m_hiArm.intakeCube(), m_hiArm))
        .onFalse(new InstantCommand(() -> m_hiArm.stopRollers(), m_hiArm));
    new Trigger(this::rightTrigger)
        .onTrue(new InstantCommand(() -> m_hiArm.expellCube(), m_hiArm))
        .onFalse(new InstantCommand(() -> m_hiArm.stopRollers(), m_hiArm));
    new JoystickButton(m_armController, Button.kLeftBumper.value)
        .onTrue(new InstantCommand(() -> m_loArm.intakeObj(), m_loArm))
        .onFalse(new InstantCommand(() -> m_loArm.stopRollers(), m_loArm));
    new Trigger(this::leftTrigger)
        .onTrue(new InstantCommand(() -> m_loArm.expellObj(), m_loArm))
        .onFalse(new InstantCommand(() -> m_loArm.stopRollers(), m_loArm));
    new JoystickButton(m_driverController, Button.kLeftBumper.value)
        .onTrue(new InstantCommand(() -> LedSubsystem.toggle_cube()));
    new JoystickButton(m_driverController, Button.kRightBumper.value)
        .onTrue(new InstantCommand(() -> LedSubsystem.toggle_cone()));
    new Trigger(this::R1Down)
        .onTrue(new InstantCommand(() -> m_hiArm.moveToPosition(0)));
    new Trigger(this::R1Up)
        .onTrue(new InstantCommand(() -> m_hiArm.moveToPosition(182)));
    // Led bar triggers
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create a voltage constraint to ensure we don't accelerate too fast
    // var autoVoltageConstraint =
    //     new DifferentialDriveVoltageConstraint(
    //         new SimpleMotorFeedforward(
    //             DriveConstants.ksVolts,
    //             DriveConstants.kvVoltSecondsPerMeter,
    //             DriveConstants.kaVoltSecondsSquaredPerMeter),
    //         DriveConstants.kDriveKinematics,
    //         5);

    // // Create config for trajectory
    // TrajectoryConfig config =
    //     new TrajectoryConfig(
    //             AutoConstants.kMaxSpeedMetersPerSecond,
    //             AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    //         // Add kinematics to ensure max speed is actually obeyed
    //         .setKinematics(DriveConstants.kDriveKinematics)
    //         // Apply the voltage constraint
    //         .addConstraint(autoVoltageConstraint);

    // // An example trajectory to follow.  All units in meters.
    // Trajectory exampleTrajectory =
    //     TrajectoryGenerator.generateTrajectory(
    //         // Start at the origin facing the +X direction
    //         new Pose2d(0, 0, new Rotation2d(0)),
    //         // Pass through these two interior waypoints, making an 's' curve path
    //         List.of(new Translation2d(1, 0)),
    //         // End 3 meters straight ahead of where we started, facing forward
    //         new Pose2d(1, 0, new Rotation2d(0)),
    //         // Pass config
    //         config);

    // /*RamseteCommand ramseteCommand =
    //     new RamseteCommand(
    //         exampleTrajectory,
    //         m_robotDrive::getPose,
    //         new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
    //         new SimpleMotorFeedforward(
    //             DriveConstants.ksVolts,
    //             DriveConstants.kvVoltSecondsPerMeter,
    //             DriveConstants.kaVoltSecondsSquaredPerMeter),
    //         DriveConstants.kDriveKinematics,
    //         m_robotDrive::getWheelSpeeds,
    //         new PIDController(DriveConstants.kPDriveVel, 0, 0),
    //         new PIDController(DriveConstants.kPDriveVel, 0, 0),
    //         // RamseteCommand passes volts to the callback
    //         m_robotDrive::tankDriveVolts,
    //         m_robotDrive);*/
    //     RamseteCommand ramseteCommand = 
    //         new RamseteCommand(exampleTrajectory, m_robotDrive::getPose, new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta), DriveConstants.kDriveKinematics, m_robotDrive::tankMetersPerSecond, m_robotDrive);
    
    m_robotDrive.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
    // Reset odometry to the starting pose of the trajectory.
    //m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose()); // for ramsete command
    SendableChooser<AutoType> type = (SendableChooser) SmartDashboard.getData("Auto Type");
    SendableChooser<AutoPiece> piece = (SendableChooser) SmartDashboard.getData("Auto Piece");
    
    if(type.getSelected().equals(AutoType.Normal)) {
        if(piece.getSelected().equals(AutoPiece.Cube)) {
            return new InstantCommand(() -> m_hiArm.moveToPosition(182), m_hiArm) //Normal Cube
                .andThen(new WaitUntilCommand(m_hiArm::atPoint))
                .andThen(() -> m_hiArm.moveRollers(0.4), m_hiArm)
                .andThen(new WaitUntilCommand(m_hiArm::notHaveCube))
                .andThen(new WaitCommand(0.5))
                .andThen(m_hiArm::stopRollers)
                .andThen(() -> m_hiArm.moveToPosition(0), m_hiArm)
                .andThen(new WaitUntilCommand(m_hiArm::atPoint))
                .andThen(new RunCommand(() -> m_robotDrive.tankMetersPerSecond(-0.6, -0.6), m_robotDrive)
                    .until(() -> m_robotDrive.getPose().minus(new Pose2d(-SmartDashboard.getNumber("Auto Distance", AutoConstants.normalAutoDistance), 0, new Rotation2d(0))).getX() <= 0))
                .andThen(new RunCommand(() -> m_robotDrive.tankMetersPerSecond(0, 0), m_robotDrive));
        }
        else {
            return new InstantCommand(() -> m_hiArm.moveToPosition(182), m_hiArm) //Normal Cone
                .andThen(() -> m_hiArm.moveRollers(0.5))
                .andThen(new WaitUntilCommand(m_hiArm::atPoint))
                .andThen(() -> m_hiArm.moveRollers(-0.4), m_hiArm)
                .andThen(new WaitCommand(1))
                .andThen(m_hiArm::stopRollers)
                .andThen(() -> m_hiArm.moveToPosition(0), m_hiArm)
                .andThen(new WaitUntilCommand(m_hiArm::atPoint))
                .andThen(new RunCommand(() -> m_robotDrive.tankMetersPerSecond(-0.6, -0.6), m_robotDrive)
                    .until(() -> m_robotDrive.getPose().minus(new Pose2d(-SmartDashboard.getNumber("Auto Distance", AutoConstants.normalAutoDistance), 0, new Rotation2d(0))).getX() <= 0))
                .andThen(new RunCommand(() -> m_robotDrive.tankMetersPerSecond(0, 0), m_robotDrive));
        }
    }
    else {
        if(piece.getSelected().equals(AutoPiece.Cube)) {
            return new InstantCommand(() -> m_hiArm.moveToPosition(182), m_hiArm) //Balencing Cube
                .andThen(new WaitUntilCommand(m_hiArm::atPoint))
                .andThen(() -> m_hiArm.moveRollers(0.4), m_hiArm)
                .andThen(new WaitUntilCommand(m_hiArm::notHaveCube))
                .andThen(new WaitCommand(0.5))
                .andThen(m_hiArm::stopRollers)
                .andThen(() -> m_hiArm.moveToPosition(0), m_hiArm)
                .andThen(new WaitUntilCommand(m_hiArm::atPoint))
                .andThen(new RunCommand(() -> m_robotDrive.tankMetersPerSecond(-0.6, -0.6), m_robotDrive)
                    .until(() -> m_robotDrive.getPose().minus(new Pose2d(-SmartDashboard.getNumber("Auto Distance", AutoConstants.balenceAutoDistance), 0, new Rotation2d(0))).getX() <= 0))
                .andThen(new ParallelRaceGroup(new RunCommand(() -> m_robotDrive.tankMetersPerSecond(0.35, 0.35), m_robotDrive), new WaitUntilCommand(m_robotDrive::onChargingStation)))
                .andThen(new ParallelRaceGroup(new RunCommand(() -> m_robotDrive.tankMetersPerSecond(0.35, 0.35), m_robotDrive), new WaitUntilCommand(m_robotDrive::balenced)))
                .andThen(new RunCommand(() -> m_robotDrive.tankMetersPerSecond(0, 0), m_robotDrive));
        }
        else {
            return new InstantCommand(() -> m_hiArm.moveToPosition(182), m_hiArm) //Balencing Cone
                .andThen(() -> m_hiArm.moveRollers(0.5))
                .andThen(new WaitUntilCommand(m_hiArm::atPoint))
                .andThen(() -> m_hiArm.moveRollers(-0.4), m_hiArm)
                .andThen(new WaitCommand(1))
                .andThen(m_hiArm::stopRollers)
                .andThen(() -> m_hiArm.moveToPosition(0), m_hiArm)
                .andThen(new WaitUntilCommand(m_hiArm::atPoint))
                .andThen(new RunCommand(() -> m_robotDrive.tankMetersPerSecond(-0.6, -0.6), m_robotDrive)
                    .until(() -> m_robotDrive.getPose().minus(new Pose2d(-SmartDashboard.getNumber("Auto Distance", AutoConstants.balenceAutoDistance), 0, new Rotation2d(0))).getX() <= 0))
                .andThen(new ParallelRaceGroup(new RunCommand(() -> m_robotDrive.tankMetersPerSecond(0.35, 0.35), m_robotDrive), new WaitUntilCommand(m_robotDrive::onChargingStation)))
                .andThen(new ParallelRaceGroup(new RunCommand(() -> m_robotDrive.tankMetersPerSecond(0.35, 0.35), m_robotDrive), new WaitUntilCommand(m_robotDrive::balenced)))
                .andThen(new RunCommand(() -> m_robotDrive.tankMetersPerSecond(0, 0), m_robotDrive));
        }
    }
    

    // Run path following command, then stop at the end.
    //return new RunCommand(() -> m_robotDrive.tankMetersPerSecond(-0.5, -0.5), m_robotDrive).until(() -> m_robotDrive.getPose().minus(new Pose2d(-1, 0, new Rotation2d(0))).getX() <= 0).andThen(() -> m_robotDrive.tankDriveVolts(0, 0));
    //4.1146 meters Normal distance Now in OIConstants
    
  }

  public Command debugAutonCommand() {
    return new RunCommand(() -> m_robotDrive.tankMetersPerSecond(1, 1), m_robotDrive);
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
    private boolean leftTrigger() {
        return m_armController.getRawAxis(2) > 0.75;
    }
    private boolean rightTrigger() {
        return m_armController.getRawAxis(3) > 0.75;
    }
    private boolean R1Down() {
        return m_armController.getRawAxis(5) > 0.75;
    }
    private boolean R1Up() {
        return m_armController.getRawAxis(5) < -0.75;
    }
    private boolean L1Down() {
        return m_armController.getRawAxis(1) > 0.75;
    }
    private boolean L1Up() {
        return m_armController.getRawAxis(1) < -0.75;
    }

    public enum AutoType {
        Normal,
        Balence
    }

    public enum AutoPiece {
        Cube,
        Cone
    }
}

