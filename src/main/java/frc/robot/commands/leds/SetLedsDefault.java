package frc.robot.commands.leds;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.SparkLedPattern;
import frc.robot.subsystems.Blinkin;

/** Instant command to set LEDs to the default (idle) color */
public class SetLedsDefault extends InstantCommand {
    public SetLedsDefault(Blinkin leds) {
        super(() -> leds.set(SparkLedPattern.GREEN), leds);
    }
}
