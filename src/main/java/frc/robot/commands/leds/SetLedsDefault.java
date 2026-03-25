package frc.robot.commands.leds;

import frc.robot.SparkLedPattern;
import frc.robot.subsystems.Blinkin;

/** Command to set LEDs to the default (idle) color using LedCommand base. */
public class SetLedsDefault extends LedCommand {
    public SetLedsDefault(Blinkin leds) {
        super(leds);
    }

    @Override
    protected SparkLedPattern getPattern() {
        return SparkLedPattern.GREEN;
    }
}
