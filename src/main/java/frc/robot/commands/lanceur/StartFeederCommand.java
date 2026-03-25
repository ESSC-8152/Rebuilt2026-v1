package frc.robot.commands.lanceur;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LanceurSubsystem;

public class StartFeederCommand extends Command{
    private final LanceurSubsystem m_lanceur;

    public StartFeederCommand(LanceurSubsystem lanceur){
        m_lanceur = lanceur;

        addRequirements(lanceur);
    }

    @Override
    public void initialize() {
        m_lanceur.startFeeder(1.0);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
