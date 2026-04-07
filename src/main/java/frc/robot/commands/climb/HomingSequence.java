package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class HomingSequence {

    public static Command getCommand(ClimberSubsystem climber) {
        return Commands.sequence(
            Commands.run(() -> climber.setSpeed(0.2), climber)
                .withTimeout(1),

            Commands.run(() -> climber.setSpeed(-0.1), climber)
                .until(climber::isHomeSwitchPressed),

            Commands.run(() -> {
                climber.setSpeed(0.1);
                climber.resetEncoder(0.0);
            }, climber)
                .until(() -> !climber.isHomeSwitchPressed()),

            Commands.runOnce(() -> {
                climber.setSpeed(0.0);
            }, climber)
        ).withName("HomingSequence");
    }
}