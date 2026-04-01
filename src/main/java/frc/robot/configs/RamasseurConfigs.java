package frc.robot.configs;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.RamasseurConstants;
import frc.robot.Constants.VortexMotorConstants;

public final class RamasseurConfigs {
    public static final SparkFlexConfig ramasseurConfig = new SparkFlexConfig();
    public static final SparkMaxConfig rotationRamasseurConfig = new SparkMaxConfig();
    
    static {
        double kVelocityFeedForward = 12 / VortexMotorConstants.kVortexMotorFreeSpeedRpm;
        double turningFactor = 2 * Math.PI;
        
        /*
            * Roues du ramasseur
            */
        ramasseurConfig
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kBrake)
            .inverted(true)
            .voltageCompensation(12.0);

        ramasseurConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(0.00008)
            .i(0.0)
            .d(0.0)
            .outputRange(-1, 1)
            .feedForward.kV(kVelocityFeedForward);

        ramasseurConfig.signals
            .primaryEncoderVelocityPeriodMs(20);

        /*
        * Rotation du ramasseur
        */
        rotationRamasseurConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(RamasseurConstants.kRotationCurrentLimit)
            .voltageCompensation(12.0);

        rotationRamasseurConfig.absoluteEncoder
            .positionConversionFactor(turningFactor) // radians
            .velocityConversionFactor(turningFactor / 60.0); // radians par seconde

        rotationRamasseurConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(0, turningFactor)

            // --- SLOT 0 : Mouvement normal
            .pid(RamasseurConstants.kRotationP, RamasseurConstants.kRotationI, RamasseurConstants.kRotationD, ClosedLoopSlot.kSlot0)
            .outputRange(-0.6, 0.6, ClosedLoopSlot.kSlot0)

            // --- SLOT 1 : Coup
            .pid(0.8, 0, 0.6, ClosedLoopSlot.kSlot1)
            .outputRange(-1, 1, ClosedLoopSlot.kSlot1);
    }
}
