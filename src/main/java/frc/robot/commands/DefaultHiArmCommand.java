package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HiArmSubsystem;

/**
 * Command that tilts the wrist up if not doing anything else
 */
public class DefaultHiArmCommand extends CommandBase {

  private final HiArmSubsystem HiarmSubsystem;

  public DefaultHiArmCommand(HiArmSubsystem HiarmSubsystem) {
    this.HiarmSubsystem = HiarmSubsystem;
    addRequirements(HiarmSubsystem);
  }

  @Override
  public void execute() {
    HiarmSubsystem.moveToPosition(0);
  }
  
  @Override
  public void end(boolean interrupted) {
    HiarmSubsystem.stop();
    HiarmSubsystem.stopRollers();
  }
}
