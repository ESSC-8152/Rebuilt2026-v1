package frc.robot.configs;

import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.LanceurConstants;
import frc.robot.Constants.VortexMotorConstants;

public final class LanceurConfigs {
    public static final SparkFlexConfig lanceurGaucheConfig = new SparkFlexConfig();
    public static final SparkFlexConfig lanceurDroitConfig = new SparkFlexConfig();
    public static final SparkFlexConfig feederConfig = new SparkFlexConfig();
    public static final SparkFlexConfig courroiesConfig = new SparkFlexConfig();
    
    static {
        double kVelocityFeedForward = 12.2 / VortexMotorConstants.kVortexMotorFreeSpeedRpm;

        // --------------------------------------------------------------
        // Configuration du lanceur (grosses roues)
        // --------------------------------------------------------------
        lanceurGaucheConfig
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(80);

        lanceurGaucheConfig.closedLoop
            .feedbackSensor(com.revrobotics.spark.FeedbackSensor.kPrimaryEncoder)
            .p(0.0005)
            .i(0.0)
            .d(0.024)
            .outputRange(-1, 1)
            .feedForward.kV(kVelocityFeedForward);

        lanceurGaucheConfig.signals
            .primaryEncoderVelocityPeriodMs(5);

        lanceurDroitConfig.follow(LanceurConstants.kMoteurGaucheLanceurID, true);

        // --------------------------------------------------------------
        // Configuration du feeder (petite roue bleue)
        // --------------------------------------------------------------
        feederConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40);

        feederConfig.closedLoop
            .feedbackSensor(com.revrobotics.spark.FeedbackSensor.kPrimaryEncoder)
            .p(0.0)
            .i(0.0)
            .d(0.0)
            .outputRange(-1, 1)
            .feedForward.kV(kVelocityFeedForward);

        feederConfig.signals
            .primaryEncoderVelocityPeriodMs(20);

        // --------------------------------------------------------------
        // Configuration des courroies (courroies du feeder)
        // --------------------------------------------------------------
        courroiesConfig
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(30)
            .inverted(false);

        courroiesConfig.closedLoop
            .feedbackSensor(com.revrobotics.spark.FeedbackSensor.kPrimaryEncoder)
            .p(0.0001)
            .i(0.0)
            .d(0.0)
            .outputRange(-1, 1)
            .feedForward.kV(kVelocityFeedForward);

        courroiesConfig.signals
            .primaryEncoderVelocityPeriodMs(20);
    }
}
