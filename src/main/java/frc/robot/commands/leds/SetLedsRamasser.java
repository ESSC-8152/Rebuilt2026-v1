package frc.robot.commands.leds;

import frc.robot.SparkLedPattern;
import frc.robot.subsystems.Blinkin;

/** Command to set LEDs to the "ramasser" (collecting) color using LedCommand base. */
public class SetLedsRamasser extends LedCommand {
    public SetLedsRamasser(Blinkin leds) {
        super(leds);
    }

    @Override
    protected SparkLedPattern getPattern() {
        return SparkLedPattern.RED;
    }
}
