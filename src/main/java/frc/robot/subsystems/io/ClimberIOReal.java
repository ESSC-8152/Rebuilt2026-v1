package frc.robot.subsystems.io;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.configs.ClimberConfigs;
import frc.robot.Constants.ClimberConstants;

public class ClimberIOReal implements ClimberIO {
    private final SparkMax climberMotor;
    private final SparkClosedLoopController climberPidController;
    private final RelativeEncoder alternateEncoder;
    private final DigitalInput homeSwitch;

    public ClimberIOReal() {
        climberMotor = new SparkMax(ClimberConstants.kMoteurClimberID, MotorType.kBrushless);

        climberPidController = climberMotor.getClosedLoopController();

        climberMotor.configure(ClimberConfigs.climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        alternateEncoder = climberMotor.getAlternateEncoder();
        homeSwitch = new DigitalInput(0);
    }

    @Override
    public void stop(){
        climberPidController.setSetpoint(0, ControlType.kDutyCycle);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.climberPositionRad = alternateEncoder.getPosition();
        inputs.climberVelocityRadPerSec = alternateEncoder.getVelocity();
        inputs.climberHomeSwitch = !homeSwitch.get();
        inputs.climberCurrentAmps = new double[]{climberMotor.getOutputCurrent()};
    }

    @Override
    public void monterClimberPositionRad(double positionRad) {
        climberPidController.setSetpoint(positionRad, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    @Override
    public void setTargetPositionRad(double positionRad, int slot, double feedforwardVolts) {
        ClosedLoopSlot revSlot = (slot == 1) ? ClosedLoopSlot.kSlot1 : ClosedLoopSlot.kSlot0;

        climberMotor.getClosedLoopController().setSetpoint(
            positionRad, 
            ControlType.kPosition, 
            revSlot, 
            feedforwardVolts, 
            ArbFFUnits.kVoltage
        );
    }

    @Override
    public void resetEncoder(double positionRad) {
        alternateEncoder.setPosition(positionRad);
    }

    @Override
    public void setSpeed(double volts) {
        climberMotor.set(volts);
    }
}
