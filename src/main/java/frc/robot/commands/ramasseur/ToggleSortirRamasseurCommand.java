package frc.robot.commands.ramasseur;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RamasseurSubsystem;

public class ToggleSortirRamasseurCommand extends Command {
    private final RamasseurSubsystem ramasseurSubsystem;

    public ToggleSortirRamasseurCommand(RamasseurSubsystem ramasseurSubsystem) {
        this.ramasseurSubsystem = ramasseurSubsystem;

        addRequirements(ramasseurSubsystem);
    }

    @Override
    public void initialize() {
        if (ramasseurSubsystem.isRamasseurDeployed) {
            ramasseurSubsystem.rentrerRamasseur();
        } else {
            ramasseurSubsystem.sortirRamasseur();
        }
    }

    @Override
    public boolean isFinished() {
        return true; // Command finishes immediately
    }
    
}
