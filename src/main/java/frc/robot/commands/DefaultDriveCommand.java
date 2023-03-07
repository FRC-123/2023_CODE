package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LedSubsystem;

public class DefaultDriveCommand extends CommandBase{
    private final DriveSubsystem driveSubsystem;
    private XboxController m_drivController = new XboxController(OIConstants.kDriverControllerPort);

    public DefaultDriveCommand(DriveSubsystem subsystem) {
        this.driveSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        double multiplier = 0.45;
        if((m_drivController.getLeftTriggerAxis() > 0.75) || (m_drivController.getRightTriggerAxis() > 0.75)) {
            multiplier = 0.7;
        }
        if(m_drivController.getPOV() == -1) {
            driveSubsystem.tankDrive(
                -multiplier*m_drivController.getLeftY(), -multiplier*m_drivController.getRightY());
        }
        else {
            if(m_drivController.getPOV() == 0) {
                driveSubsystem.tankDrive(0.25, 0.25);
            }
            else if(m_drivController.getPOV() == 45) {
                driveSubsystem.tankDrive(0.3, -0.12);
            }
            else if(m_drivController.getPOV() == 90) {
                driveSubsystem.tankDrive(0.3, -0.3);
            }
            else if(m_drivController.getPOV() == 135) {
                driveSubsystem.tankDrive(-0.3, 0.12);
            }
            else if(m_drivController.getPOV() == 180) {
                driveSubsystem.tankDrive(-0.25, -0.25);
            }
            else if(m_drivController.getPOV() == 225) {
                driveSubsystem.tankDrive(0.12, -0.3);
            }
            else if(m_drivController.getPOV() == 270) {
                driveSubsystem.tankDrive(-0.3, 0.3);
            }
            else if(m_drivController.getPOV() == 315) {
                driveSubsystem.tankDrive(-0.12, 0.3);
            }
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
