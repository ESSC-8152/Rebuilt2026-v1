package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LanceurConstants;
import frc.robot.configs.LanceurConfigs;

public class LanceurSubsystem extends SubsystemBase {
    private final SparkFlex moteurGaucheLanceur;
    private final SparkFlex moteurDroitLanceur;
    private final SparkFlex moteurCourroies;
    private final SparkFlex moteurFeeder;

    private final SparkClosedLoopController courroiesPidController;
    private final SparkClosedLoopController lanceurPidController;
    private final SparkClosedLoopController feederPidController;



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

        // --------------------------------------------------------------
        // Configuration du lanceur (grosses roues)
        // --------------------------------------------------------------
        moteurGaucheLanceur.configure(
            LanceurConfigs.LanceurSubsystem.lanceurGaucheConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        moteurDroitLanceur.configure(
            LanceurConfigs.LanceurSubsystem.lanceurDroitConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        // --------------------------------------------------------------
        // Configuration du feeder (petite roue bleue)
        // --------------------------------------------------------------
        moteurFeeder.configure(
            LanceurConfigs.LanceurSubsystem.feederConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        // --------------------------------------------------------------
        // Configuration des courroies (courroies du feeder)
        // --------------------------------------------------------------
        moteurCourroies.configure(
            LanceurConfigs.LanceurSubsystem.courroiesConfig,
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