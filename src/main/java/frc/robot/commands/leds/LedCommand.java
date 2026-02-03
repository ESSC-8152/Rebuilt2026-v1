package frc.robot.commands.leds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SparkLedPattern;
import frc.robot.subsystems.Blinkin;

/**
 * Abstract base class for simple LED commands.
 *
 * Subclasses should implement {@link #getPattern()} to return the desired
 * {@link SparkLedPattern}. This base class handles setting the pattern on
 * initialize and finishing immediately (instant behavior).
 */
public abstract class LedCommand extends Command {
    protected final Blinkin leds;

    protected LedCommand(Blinkin leds) {
        this.leds = leds;
        addRequirements(leds);
    }

    /** Return the pattern that will be applied when this command is initialized. */
    protected abstract SparkLedPattern getPattern();

    @Override
    public void initialize() {
        if (leds != null) {
            leds.set(getPattern());
        }
    }

    @Override
    public boolean isFinished() {
        return true; // instant command behavior
    }
}
