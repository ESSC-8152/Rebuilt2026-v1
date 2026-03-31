package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LanceurConstants;
import frc.robot.subsystems.io.LanceurIO;
import frc.robot.subsystems.io.LanceurIOInputsAutoLogged;

public class LanceurSubsystem extends SubsystemBase {
    private final LanceurIO io;
    private final LanceurIOInputsAutoLogged inputs = new LanceurIOInputsAutoLogged();

    private boolean lanceurEnMarche = false;

    public LanceurSubsystem(LanceurIO io) {
        this.io = io;
        startLanceurLent();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Lanceur", inputs);
    }

    public void toggleLanceur() {
        if (lanceurEnMarche) {
            startLanceurLent();
        } else {
            io.setLanceurVelocityRPM(LanceurConstants.kVitesseLanceur);
            lanceurEnMarche = true;
        }
    }

    public void startLanceur(){
        lanceurPidController.setSetpoint(LanceurConstants.kVitesseLanceur, ControlType.kVelocity);
        lanceurEnMarche = true;
    }    

    public void startLanceurLent() {
        lanceurPidController.setSetpoint(LanceurConstants.kVitesseLanceurLent, ControlType.kVelocity);
        lanceurEnMarche = false;
    }

    public void startFeeder(double speed) {
        io.setFeederVelocityRPM(LanceurConstants.kVitesseFeeder * speed, LanceurConstants.kVitesseCourroies * speed);
    }

    public void arreterFeeder() {
        io.stopFeeder();
    }
}