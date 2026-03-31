package frc.robot.subsystems.io;

public class RamasseurIOSim implements RamasseurIO {
    private double ramasseurRpm = 0.0;
    private double rotationPositionRad = 0.0;

    @Override
    public void updateInputs(RamasseurIOInputs inputs) {
        inputs.ramasseurVelocityRpm = ramasseurRpm;
        inputs.rotationAbsolutePositionRad = rotationPositionRad;
    }

    @Override
    public void setRamasseurVelocityRPM(double rpmTarget) {
        ramasseurRpm = rpmTarget;
    }

    @Override
    public void setRotationPosition(double targetPosition, int closedLoopSlot) {
        rotationPositionRad = targetPosition;
    }

    @Override
    public void stop() {
        ramasseurRpm = 0.0;
    }
}