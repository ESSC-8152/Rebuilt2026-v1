package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.io.ClimberIO;
import frc.robot.subsystems.io.ClimberIOInputsAutoLogged;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
    private ClimberIO io;
    private ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    private boolean estGrimpe = false;

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);
    }

    public ClimberSubsystem(ClimberIO climberIO) {
        this.io = climberIO;
    }

    public void resetEncoder(double positionRad) {
        io.resetEncoder(positionRad);
    }

    public void toggleMonter(){
        if (estGrimpe){
            descendreClimber();
        } else {
            monterClimber();
        }
    }

    public void monterClimber(){
        io.setTargetPositionRad(ClimberConstants.kClimberUpPosition, 0, 0.0);
        estGrimpe = true;
    }

    public void descendreClimber() {
        io.setTargetPositionRad(ClimberConstants.kClimberAccrochePosition, 1, 0);
        estGrimpe = false;
    }

    public void setSpeed(double volts) {
        io.setSpeed(volts);
    }

    public void stop(){
        io.stop();
    }

    public boolean isHomeSwitchPressed() {
        estGrimpe = false;
        return inputs.climberHomeSwitch;
    }

    public Trigger getHomeSwitchTrigger() {
        return new Trigger(this::isHomeSwitchPressed);
    }

    public double getVelocityRadPerSec() {
        return inputs.climberVelocityRadPerSec;
    }

    public double getCurrentAmps() {
        return inputs.climberCurrentAmps.length > 0 ? inputs.climberCurrentAmps[0] : 0.0;
    }
}