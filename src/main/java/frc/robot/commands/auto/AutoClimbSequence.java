package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.drive.AlignToPosePIDCommand;
import frc.robot.commands.drive.DriveToPoseCommand;
import frc.robot.subsystems.DriveSubsystem;

public class AutoClimbSequence {
    
    public static Command getClimbSequence(DriveSubsystem drive) {
        return Commands.sequence(
            DriveToPoseCommand.create(AutoConstants.kClimbPose)
                .until(() -> {
                    double distance = drive.getPose().getTranslation().getDistance(AutoConstants.kClimbPose.getTranslation());
                    return distance < 1.20; 
                }),

            new AlignToPosePIDCommand(drive, AutoConstants.kClimbPose)
        ).withName("Auto Climb Sequence");
    }
}
