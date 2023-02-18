package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LoArmSubsystem;

/**
 * Command that tilts the wrist up if not doing anything else
 */
public class DefaultLoArmCommand extends CommandBase {

  private final LoArmSubsystem LoarmSubsystem;

  public DefaultLoArmCommand(LoArmSubsystem LoarmSubsystem) {
    this.LoarmSubsystem = LoarmSubsystem;
    addRequirements(LoarmSubsystem);
  }

  @Override
  public void execute() {
    LoarmSubsystem.moveToPosition(0);
  }
  
  @Override
  public void end(boolean interrupted) {
    LoarmSubsystem.stop();
    LoarmSubsystem.stopRollers();
  }
}
