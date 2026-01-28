package frc.robot.commands.leds;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.SparkLedPattern;
import frc.robot.subsystems.Blinkin;

/** Instant command to set LEDs to the tracking pattern */
public class SetLedsTracking extends InstantCommand {
    public SetLedsTracking(Blinkin leds) {
        super(() -> leds.set(SparkLedPattern.COLOR1_STROBE), leds);
    }
}
