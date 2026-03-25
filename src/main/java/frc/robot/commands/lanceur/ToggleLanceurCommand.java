package frc.robot.commands.lanceur;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LanceurSubsystem;

public class ToggleLanceurCommand extends Command{
    private final LanceurSubsystem lanceurSubsystem;

    public ToggleLanceurCommand(LanceurSubsystem lanceurSubsystem){
        this.lanceurSubsystem = lanceurSubsystem;
        addRequirements(lanceurSubsystem);
    }

    @Override
    public void initialize() {
        lanceurSubsystem.toggleLanceur();
    }

     @Override
     public boolean isFinished() {
         return true;
     }
    
}
