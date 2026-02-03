package frc.robot.commands.leds;

import frc.robot.SparkLedPattern;
import frc.robot.subsystems.Blinkin;

/** Command to set LEDs to the tracking pattern using LedCommand base. */
public class SetLedsTracking extends LedCommand {
    public SetLedsTracking(Blinkin leds) {
        super(leds);
    }

    @Override
    protected SparkLedPattern getPattern() {
        return SparkLedPattern.COLOR1_STROBE;
    }
}
