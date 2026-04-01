package frc.robot.configs;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

public final class ClimberConfigs {
    public static final SparkMaxConfig climberConfig = new SparkMaxConfig();

    static {
        double turningFactor = 2 * Math.PI; //Radians
        double velocityConversionFactor = turningFactor / 60.0; //radians par seconde

        climberConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40);

        climberConfig.alternateEncoder
            .countsPerRevolution(8192)
            .positionConversionFactor(turningFactor)
            .velocityConversionFactor(velocityConversionFactor);

        climberConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
            .pid(0.2, 0.0, 0.0)
            .outputRange(-1, 1);
    }
}
