package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class SetBoostModeCommand extends Command{
    private final DriveSubsystem drive;
    private final boolean boost;

    public SetBoostModeCommand(DriveSubsystem drive, boolean boost) {
        this.drive = drive;
        this.boost = boost;
    }

    @Override
    public void initialize() {
        drive.setBoostMode(boost);
    }

     @Override
     public boolean isFinished() {
         return true; // Commande instantanée
     }
}
