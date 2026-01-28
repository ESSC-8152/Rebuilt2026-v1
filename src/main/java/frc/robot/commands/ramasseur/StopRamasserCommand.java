package frc.robot.commands.ramasseur;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RamasseurSubsystem;

public class StopRamasserCommand extends Command{
private final RamasseurSubsystem m_ramasseur;

    public StopRamasserCommand(RamasseurSubsystem ramasseur) {
        m_ramasseur = ramasseur;

        addRequirements(ramasseur);
    }

    @Override
    public void initialize() {
        m_ramasseur.stop();
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
