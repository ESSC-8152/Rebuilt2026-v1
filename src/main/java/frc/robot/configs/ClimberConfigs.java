package frc.robot.configs;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

public final class ClimberConfigs {
    public static final SparkMaxConfig climberConfig = new SparkMaxConfig();

    static {
        double turningFactor = 2 * Math.PI; //Radians
        double velocityConversionFactor = turningFactor / 60.0; //radians par seconde

        climberConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40);

        climberConfig.alternateEncoder
            .setSparkMaxDataPortConfig()
            .countsPerRevolution(8192)
            .positionConversionFactor(turningFactor)
            .velocityConversionFactor(velocityConversionFactor);

        climberConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
            .outputRange(-1, 1)

            .pid(0.25, 0.0, 0.0, ClosedLoopSlot.kSlot0)
            .allowedClosedLoopError(0.25, ClosedLoopSlot.kSlot0)

            .outputRange(-0.6, 0.6, ClosedLoopSlot.kSlot1) 
            .pid(0.07, 0.0005, 0.0, ClosedLoopSlot.kSlot1) 
            .allowedClosedLoopError(0.25, ClosedLoopSlot.kSlot1);
    }
}
