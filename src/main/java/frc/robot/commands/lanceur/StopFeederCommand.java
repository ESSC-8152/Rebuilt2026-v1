package frc.robot.commands.lanceur;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LanceurSubsystem;

public class StopFeederCommand extends Command{
    private final LanceurSubsystem m_lanceur;

    public StopFeederCommand(LanceurSubsystem lanceur){
        m_lanceur = lanceur;

        addRequirements(lanceur);
    }

    @Override
    public void initialize() {
        m_lanceur.arreterFeeder();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
