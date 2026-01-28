package frc.robot.commands.leds;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.SparkLedPattern;
import frc.robot.subsystems.Blinkin;

/** Instant command to set LEDs to the "ramasser" (collecting) color */
public class SetLedsRamasser extends InstantCommand {
    public SetLedsRamasser(Blinkin leds) {
        super(() -> leds.set(SparkLedPattern.RED), leds);
    }
}
