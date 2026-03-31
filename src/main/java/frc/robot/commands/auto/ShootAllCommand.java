package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ramasseur.kickRamasseurCommand;
import frc.robot.subsystems.LanceurSubsystem;
import frc.robot.subsystems.RamasseurSubsystem;

public class ShootAllCommand extends Command{
    private final LanceurSubsystem m_lanceur;
    private final RamasseurSubsystem m_ramasseur;
    private final kickRamasseurCommand m_kickRamasseurCommand;

    private final Timer _timer = new Timer();
    private boolean kick = false;

    public ShootAllCommand(LanceurSubsystem lanceur, RamasseurSubsystem ramasseur) {
        m_lanceur = lanceur;
        m_ramasseur = ramasseur;

        m_kickRamasseurCommand = new kickRamasseurCommand(ramasseur);

        addRequirements(lanceur, ramasseur);
    }

    @Override
    public void initialize(){
        m_lanceur.startLanceur();
        m_lanceur.startFeeder(1.0);
        m_ramasseur.ramasser();

        _timer.reset();
        _timer.start();
    }

    @Override
    public void execute(){
        if (!kick && _timer.hasElapsed(1.0)) {
            m_ramasseur.kickRamasseur();
            _timer.reset();
            kick = true;
        }

        if (kick && _timer.hasElapsed(1.0)) { 
            m_ramasseur.sortirRamasseur();
            _timer.reset();
            kick = false;
        }
    }

    @Override
    public void end(boolean interrupted){
        m_lanceur.startLanceurLent();
        m_lanceur.arreterFeeder();
        m_ramasseur.sortirRamasseur();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}