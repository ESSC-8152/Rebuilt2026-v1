package frc.robot.subsystems.io;

import org.littletonrobotics.junction.AutoLog;

public interface LanceurIO {
    @AutoLog
    public static class LanceurIOInputs {
        public double lanceurGaucheVelocityRpm = 0.0;
        public double lanceurGaucheAppliedVolts = 0.0;
        public double[] lanceurGaucheCurrentAmps = new double[]{};

        public double lanceurDroitVelocityRpm = 0.0;
        public double lanceurDroitAppliedVolts = 0.0;
        public double[] lanceurDroitCurrentAmps = new double[]{};

        public double feederVelocityRpm = 0.0;
        public double feederAppliedVolts = 0.0;
        public double[] feederCurrentAmps = new double[]{};

        public double courroiesVelocityRpm = 0.0;
        public double courroiesAppliedVolts = 0.0;
        public double[] courroiesCurrentAmps = new double[]{};
    }

    public default void updateInputs(LanceurIOInputs inputs) {}

    /** Assigne une vitesse cible en RPM pour le lanceur (gauche et droite) */
    public default void setLanceurVelocityRPM(double rpmTarget) {}

    /** Assigne une vitesse cible en RPM pour le feeder et les courroies */
    public default void setFeederVelocityRPM(double feederRpm, double courroiesRpm) {}

    /** Coupe l'alimentation du feeder et des courroies */
    public default void stopFeeder() {}
}