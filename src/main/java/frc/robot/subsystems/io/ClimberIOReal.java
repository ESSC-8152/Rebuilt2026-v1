package frc.robot.subsystems.io;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.configs.ClimberConfigs;
import frc.robot.Constants.ClimberConstants;

public class ClimberIOReal implements ClimberIO {
    private final SparkMax climberMotor;

    private final SparkClosedLoopController climberPidController;

    public ClimberIOReal() {
        climberMotor = new SparkMax(ClimberConstants.kMoteurClimberID, MotorType.kBrushless);

        climberPidController = climberMotor.getClosedLoopController();

        climberMotor.configure(ClimberConfigs.climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.climberPositionRad = climberMotor.getEncoder().getPosition();
        inputs.climberVelocityRadPerSec = climberMotor.getEncoder().getVelocity();
    }

    @Override
    public void setClimberPositionRad(double positionRad) {
        climberPidController.setSetpoint(positionRad, ControlType.kPosition);
    }
}
