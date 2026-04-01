package frc.robot.subsystems.io;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.LanceurConstants;
import frc.robot.configs.LanceurConfigs;

public class LanceurIOReal implements LanceurIO {
    private final SparkFlex moteurGaucheLanceur;
    private final SparkFlex moteurDroitLanceur;
    private final SparkFlex moteurCourroies;
    private final SparkFlex moteurFeeder;

    private final SparkClosedLoopController courroiesPidController;
    private final SparkClosedLoopController lanceurPidController;
    private final SparkClosedLoopController feederPidController;

    public LanceurIOReal() {
        moteurGaucheLanceur = new SparkFlex(LanceurConstants.kMoteurGaucheLanceurID, MotorType.kBrushless);
        moteurDroitLanceur = new SparkFlex(LanceurConstants.kMoteurDroitLanceurID, MotorType.kBrushless);
        moteurFeeder = new SparkFlex(LanceurConstants.kMoteurFeederID, MotorType.kBrushless);
        moteurCourroies = new SparkFlex(LanceurConstants.kMoteurFeederBaseID, MotorType.kBrushless);

        courroiesPidController = moteurCourroies.getClosedLoopController();
        feederPidController = moteurFeeder.getClosedLoopController();
        lanceurPidController = moteurGaucheLanceur.getClosedLoopController();

        moteurGaucheLanceur.configure(LanceurConfigs.lanceurGaucheConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        moteurDroitLanceur.configure(LanceurConfigs.lanceurDroitConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        moteurFeeder.configure(LanceurConfigs.feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        moteurCourroies.configure(LanceurConfigs.courroiesConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(LanceurIOInputs inputs) {
        inputs.lanceurGaucheVelocityRpm = moteurGaucheLanceur.getEncoder().getVelocity();
        inputs.lanceurGaucheAppliedVolts = moteurGaucheLanceur.getAppliedOutput() * moteurGaucheLanceur.getBusVoltage();
        inputs.lanceurGaucheCurrentAmps = new double[]{moteurGaucheLanceur.getOutputCurrent()};

        inputs.lanceurDroitVelocityRpm = moteurDroitLanceur.getEncoder().getVelocity();
        inputs.lanceurDroitAppliedVolts = moteurDroitLanceur.getAppliedOutput() * moteurDroitLanceur.getBusVoltage();
        inputs.lanceurDroitCurrentAmps = new double[]{moteurDroitLanceur.getOutputCurrent()};

        inputs.feederVelocityRpm = moteurFeeder.getEncoder().getVelocity();
        inputs.feederAppliedVolts = moteurFeeder.getAppliedOutput() * moteurFeeder.getBusVoltage();
        inputs.feederCurrentAmps = new double[]{moteurFeeder.getOutputCurrent()};

        inputs.courroiesVelocityRpm = moteurCourroies.getEncoder().getVelocity();
        inputs.courroiesAppliedVolts = moteurCourroies.getAppliedOutput() * moteurCourroies.getBusVoltage();
        inputs.courroiesCurrentAmps = new double[]{moteurCourroies.getOutputCurrent()};
    }

    @Override
    public void setLanceurVelocityRPM(double rpmTarget) {
        lanceurPidController.setSetpoint(rpmTarget, ControlType.kVelocity);
    }

    @Override
    public void setFeederVelocityRPM(double feederRpm, double courroiesRpm) {
        feederPidController.setSetpoint(feederRpm, ControlType.kVelocity);
        courroiesPidController.setSetpoint(courroiesRpm, ControlType.kVelocity);
    }

    @Override
    public void stopFeeder() {
        moteurFeeder.stopMotor();
        moteurCourroies.stopMotor();
    }
}