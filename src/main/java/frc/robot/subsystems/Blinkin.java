package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SparkLedPattern;
import frc.robot.Constants.LedConstants;

/**
 * Simple LED wrapper for a Spark-based Blinkin controller.
 *
 * Provides a small, explicit API that accepts the project's {@link SparkLedPattern}
 * enum to make calls clearer and avoid passing raw doubles throughout the codebase.
 */
public class Blinkin extends SubsystemBase {
	private final Spark blinkin;

	public Blinkin() {
		// Use a constant so the port is configurable from a single place
		blinkin = new Spark(LedConstants.kBlinkinPwmPort);
	}

	/** Set by a raw value (keeps backward compatibility). */
	public void set(double value) {
		blinkin.set(value);
	}

	/** Set using the typed SparkLedPattern enum for clarity. */
	public void set(SparkLedPattern pattern) {
		if (pattern == null) return;
		blinkin.set(pattern.getValue());
	}
}