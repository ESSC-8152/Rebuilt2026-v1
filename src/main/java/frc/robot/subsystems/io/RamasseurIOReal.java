package frc.robot.subsystems.io;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.RamasseurConstants;
import frc.robot.configs.RamasseurConfigs;

public class RamasseurIOReal implements RamasseurIO {
    private final SparkFlex moteurRamasseur;
    private final SparkMax moteurRotationRamasseur;

    private final SparkClosedLoopController ramasseurPidController;
    private final SparkClosedLoopController rotationRamasseurPidController;

    public RamasseurIOReal() {
        moteurRamasseur = new SparkFlex(RamasseurConstants.kMoteurRamasseurID, MotorType.kBrushless);
        moteurRotationRamasseur = new SparkMax(RamasseurConstants.kMoteurRotationRamasseurID, MotorType.kBrushless);

        ramasseurPidController = moteurRamasseur.getClosedLoopController();
        rotationRamasseurPidController = moteurRotationRamasseur.getClosedLoopController();

        moteurRamasseur.configure(RamasseurConfigs.ramasseurConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        moteurRotationRamasseur.configure(RamasseurConfigs.rotationRamasseurConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(RamasseurIOInputs inputs) {
        inputs.ramasseurVelocityRpm = moteurRamasseur.getEncoder().getVelocity();
        inputs.ramasseurAppliedVolts = moteurRamasseur.getAppliedOutput() * moteurRamasseur.getBusVoltage();
        inputs.ramasseurCurrentAmps = new double[]{moteurRamasseur.getOutputCurrent()};

        inputs.rotationAbsolutePositionRad = moteurRotationRamasseur.getAbsoluteEncoder().getPosition();
        inputs.rotationVelocityRadPerSec = moteurRotationRamasseur.getAbsoluteEncoder().getVelocity();
        inputs.rotationAppliedVolts = moteurRotationRamasseur.getAppliedOutput() * moteurRotationRamasseur.getBusVoltage();
        inputs.rotationCurrentAmps = new double[]{moteurRotationRamasseur.getOutputCurrent()};
    }

    @Override
    public void setRamasseurVelocityRPM(double rpmTarget) {
        ramasseurPidController.setSetpoint(rpmTarget, ControlType.kVelocity);
    }

    @Override
    public void setRotationPosition(double targetPosition, int closedLoopSlot) {
        ClosedLoopSlot slot = (closedLoopSlot == 1) ? ClosedLoopSlot.kSlot1 : ClosedLoopSlot.kSlot0;
        rotationRamasseurPidController.setSetpoint(targetPosition, ControlType.kPosition, slot);
    }

    @Override
    public void stop() {
        moteurRamasseur.stopMotor();
    }
}