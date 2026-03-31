package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RamasseurConstants;
import frc.robot.subsystems.io.RamasseurIO;
import frc.robot.subsystems.io.RamasseurIOInputsAutoLogged;

public class RamasseurSubsystem extends SubsystemBase {
    private final RamasseurIO io;
    private final RamasseurIOInputsAutoLogged inputs = new RamasseurIOInputsAutoLogged();

    public boolean isRamasseurDeployed = false;
    private double targetPosition = 0;

    public RamasseurSubsystem(RamasseurIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Ramasseur", inputs);
    }

    public void ramasser() {
        io.setRamasseurVelocityRPM(RamasseurConstants.kVitesseRamasseur);
    }

    public void rentrerRamasseur() {
        isRamasseurDeployed = false;
        targetPosition = RamasseurConstants.kRetractedPosition;
        io.setRotationPosition(targetPosition, 0);
    }

    public void sortirRamasseur() {
        isRamasseurDeployed = true;
        targetPosition = RamasseurConstants.kExtendedPosition;
        io.setRotationPosition(targetPosition, 0);
    }

    public void kickRamasseur() {
        if (!isRamasseurDeployed) {
            return;
        }

        targetPosition = RamasseurConstants.kMidPosition;
        io.setRotationPosition(targetPosition, 1);
    }

    public boolean isAtPosition() {
        double current = inputs.rotationAbsolutePositionRad;
        return Math.abs(targetPosition - current) < 0.1;
    }

    public void stop() {
        io.stop();
    }
}