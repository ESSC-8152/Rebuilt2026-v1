package frc.robot.subsystems.io;

public class LanceurIOSim implements LanceurIO {
    private double lanceurRpm = 0.0;
    private double feederRpm = 0.0;
    private double courroiesRpm = 0.0;

    @Override
    public void updateInputs(LanceurIOInputs inputs) {
        inputs.lanceurGaucheVelocityRpm = lanceurRpm;
        inputs.lanceurDroitVelocityRpm = lanceurRpm;
        inputs.feederVelocityRpm = feederRpm;
        inputs.courroiesVelocityRpm = courroiesRpm;
    }

    @Override
    public void setLanceurVelocityRPM(double rpmTarget) {
        lanceurRpm = rpmTarget;
    }

    @Override
    public void setFeederVelocityRPM(double targetFeeder, double targetCourroies) {
        feederRpm = targetFeeder;
        courroiesRpm = targetCourroies;
    }

    @Override
    public void stopFeeder() {
        feederRpm = 0.0;
        courroiesRpm = 0.0;
    }
}