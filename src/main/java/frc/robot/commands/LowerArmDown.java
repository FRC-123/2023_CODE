package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HiArmSubsystem;
import frc.robot.subsystems.LoArmSubsystem;

public class LowerArmDown extends CommandBase{
    private HiArmSubsystem m_hiarm;
    private LoArmSubsystem m_loarm;

    public LowerArmDown(HiArmSubsystem hi, LoArmSubsystem lo) {
        m_hiarm = hi;
        m_loarm = lo;
    }

    @Override
    public void initialize() {
        
    }

}
