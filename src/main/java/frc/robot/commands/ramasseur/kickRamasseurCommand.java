package frc.robot.commands.ramasseur;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.RamasseurSubsystem;

public class kickRamasseurCommand extends Command{
    private final RamasseurSubsystem m_ramasseur;

    public kickRamasseurCommand(RamasseurSubsystem m_ramasseur){
        this.m_ramasseur = m_ramasseur;

        addRequirements(m_ramasseur);
    }

    @Override
    public void initialize(){
        m_ramasseur.kickRamasseur();
    }

    @Override
    public void end(boolean interrupted){
        CommandScheduler.getInstance().schedule(Commands.sequence(
            new WaitCommand(0.25),
            new InstantCommand(() -> m_ramasseur.sortirRamasseur())
        ));
    }

    @Override
    public boolean isFinished() {
        return m_ramasseur.isAtPosition();
    }
}
