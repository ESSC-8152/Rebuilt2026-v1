package frc.robot.commands.ramasseur;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RamasseurSubsystem;

public class kickRamasseurPasSortir extends Command{
    private final RamasseurSubsystem m_ramasseur;

    public kickRamasseurPasSortir(RamasseurSubsystem m_ramasseur){
        this.m_ramasseur = m_ramasseur;

        addRequirements(m_ramasseur);
    }

    @Override
    public void initialize(){
        m_ramasseur.kickRamasseur();
    }

    @Override
    public void end(boolean interrupted){
        m_ramasseur.sortirRamasseur();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
