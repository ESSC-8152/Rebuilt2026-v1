package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LanceurSubsystem extends SubsystemBase{
    private final SparkFlex moteurGaucheLanceur;
    private final SparkFlex moteurDroitLanceur;

    private final SparkFlex moteurFeeder;

    private final SparkClosedLoopController lanceurPidController;
    private final SparkClosedLoopController feederPidController;

    private final SparkFlexConfig lanceurConfig;
    private final SparkFlexConfig feederConfig;

    public LanceurSubsystem(){
        moteurGaucheLanceur = new SparkFlex(11, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
        moteurDroitLanceur = new SparkFlex(12, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);

        moteurFeeder = new SparkFlex(13, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);

        feederPidController = moteurFeeder.getClosedLoopController();
        lanceurPidController = moteurGaucheLanceur.getClosedLoopController();

        lanceurConfig = new SparkFlexConfig();
        feederConfig = new SparkFlexConfig();

        // --------------------------------------------------------------
        // Configuration du lanceur (grosses roues)
        // --------------------------------------------------------------
        
        lanceurConfig
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(30);

        lanceurConfig.closedLoop
            .feedbackSensor(com.revrobotics.spark.FeedbackSensor.kPrimaryEncoder)
            .p(0.0)
            .i(0.0)
            .d(0.0)
            .outputRange(-1, 1)
            .feedForward.kV(0.00016);

        lanceurConfig.signals
            .primaryEncoderVelocityPeriodMs(20);

        moteurGaucheLanceur.configure(lanceurConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        lanceurConfig.follow(moteurGaucheLanceur, true);

        moteurDroitLanceur.configure(lanceurConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //--------------------------------------------------------------
        // Configuration du feeder (petite roue bleue)
        //--------------------------------------------------------------
        feederConfig
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(30);

        feederConfig.closedLoop
            .feedbackSensor(com.revrobotics.spark.FeedbackSensor.kPrimaryEncoder)
            .p(0.0)
            .i(0.0)
            .d(0.0)
            .outputRange(-1, 1)
            .feedForward.kV(0.00016);

        feederConfig.signals
            .primaryEncoderVelocityPeriodMs(20);

        moteurFeeder.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void startLanceur(){
        lanceurPidController.setSetpoint(100, ControlType.kVelocity);
    }

    public void arreterLanceur(){
        lanceurPidController.setSetpoint(0, ControlType.kVelocity);
    }

    public void startFeeder(){
        feederPidController.setSetpoint(-3000, ControlType.kVelocity);
    }

    public void arreterFeeder(){
        feederPidController.setSetpoint(0, ControlType.kVelocity);
    }
}