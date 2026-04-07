package frc.robot.commands.lanceur;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.LanceurSubsystem;

public class ToggleViteModeCommand extends Command{
    private final LanceurSubsystem m_lanceur;

    public ToggleViteModeCommand(LanceurSubsystem lanceur){
        m_lanceur = lanceur;
    }

    @Override
    public void initialize(){
        m_lanceur.toggleViteMode();
    }
}
