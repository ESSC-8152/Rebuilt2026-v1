package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LanceurConstants;

public class LanceurSubsystem extends SubsystemBase {
    private final SparkFlex moteurGaucheLanceur;
    private final SparkFlex moteurDroitLanceur;
    private final SparkFlex moteurCourroies;
    private final SparkFlex moteurFeeder;

    private final SparkClosedLoopController courroiesPidController;
    private final SparkClosedLoopController lanceurPidController;
    private final SparkClosedLoopController feederPidController;

    private final SparkFlexConfig lanceurConfig;
    private final SparkFlexConfig feederConfig;
    private final SparkFlexConfig courroiesConfig;

    private boolean lanceurEnMarche = false;

    public LanceurSubsystem() {
        moteurGaucheLanceur = new SparkFlex(
            LanceurConstants.kMoteurGaucheLanceurID,
            com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless
        );
        moteurDroitLanceur = new SparkFlex(
            LanceurConstants.kMoteurDroitLanceurID,
            com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless
        );

        moteurFeeder = new SparkFlex(
            LanceurConstants.kMoteurFeederID,
            com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless
        );
        moteurCourroies = new SparkFlex(
            LanceurConstants.kMoteurFeederBaseID,
            com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless
        );

        courroiesPidController = moteurCourroies.getClosedLoopController();
        feederPidController = moteurFeeder.getClosedLoopController();
        lanceurPidController = moteurGaucheLanceur.getClosedLoopController();

        lanceurConfig = new SparkFlexConfig();
        feederConfig = new SparkFlexConfig();
        courroiesConfig = new SparkFlexConfig();

        // --------------------------------------------------------------
        // Configuration du lanceur (grosses roues)
        // --------------------------------------------------------------
        lanceurConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(60)
            .voltageCompensation(12.0);

        lanceurConfig.closedLoop
            .feedbackSensor(com.revrobotics.spark.FeedbackSensor.kPrimaryEncoder)
            .p(0.00001)
            .i(0.0)
            .d(0.0)
            .outputRange(-1, 1)
            .feedForward.kV(0.001769);

        lanceurConfig.signals
            .primaryEncoderVelocityPeriodMs(20);

        moteurGaucheLanceur.configure(
            lanceurConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        lanceurConfig.follow(moteurGaucheLanceur, true);

        moteurDroitLanceur.configure(
            lanceurConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        // --------------------------------------------------------------
        // Configuration du feeder (petite roue bleue)
        // --------------------------------------------------------------
        feederConfig
            .idleMode(IdleMode.kBrake)
            .voltageCompensation(12.0)
            .smartCurrentLimit(40);

        feederConfig.closedLoop
            .feedbackSensor(com.revrobotics.spark.FeedbackSensor.kPrimaryEncoder)
            .p(0.0)
            .i(0.0)
            .d(0.0)
            .outputRange(-1, 1)
            .feedForward.kV(0.001769);

        feederConfig.signals
            .primaryEncoderVelocityPeriodMs(20);

        moteurFeeder.configure(
            feederConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        // --------------------------------------------------------------
        // Configuration des courroies (courroies du feeder)
        // --------------------------------------------------------------
        courroiesConfig
            .idleMode(IdleMode.kCoast)
            .voltageCompensation(12.0)
            .smartCurrentLimit(30)
            .inverted(true);

        courroiesConfig.closedLoop
            .feedbackSensor(com.revrobotics.spark.FeedbackSensor.kPrimaryEncoder)
            .p(0.0001)
            .i(0.0)
            .d(0.0)
            .outputRange(-1, 1)
            .feedForward.kV(0.001769);

        courroiesConfig.signals
            .primaryEncoderVelocityPeriodMs(20);

        moteurCourroies.configure(
            courroiesConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        startLanceurLent();
    }

    @Override
    public void periodic(){
        double vitesseLanceur = moteurGaucheLanceur.getEncoder().getVelocity();

        SmartDashboard.putNumber("Lanceur RPM", vitesseLanceur);
    }

    public void toggleLanceur() {
        if (lanceurEnMarche) {
            startLanceurLent();
        } else {
            lanceurPidController.setSetpoint(LanceurConstants.kVitesseLanceur, ControlType.kVelocity);
            lanceurEnMarche = true;
        }
    }

    public void startFeeder(double speed) {
        feederPidController.setSetpoint(LanceurConstants.kVitesseFeeder * speed, ControlType.kVelocity);
        courroiesPidController.setSetpoint(LanceurConstants.kVitesseCourroies * speed, ControlType.kVelocity);
    }

    public void startLanceurLent() {
        lanceurPidController.setSetpoint(LanceurConstants.kVitesseLanceurLent, ControlType.kVelocity);
        lanceurEnMarche = false;
    }

    public void arreterFeeder() {
        moteurFeeder.stopMotor();
        moteurCourroies.stopMotor();
    }
}