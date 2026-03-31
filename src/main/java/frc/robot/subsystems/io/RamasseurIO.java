package frc.robot.subsystems.io;

import org.littletonrobotics.junction.AutoLog;

public interface RamasseurIO {
    @AutoLog
    public static class RamasseurIOInputs {
        public double ramasseurVelocityRpm = 0.0;
        public double ramasseurAppliedVolts = 0.0;
        public double[] ramasseurCurrentAmps = new double[]{};

        public double rotationAbsolutePositionRad = 0.0;
        public double rotationVelocityRadPerSec = 0.0;
        public double rotationAppliedVolts = 0.0;
        public double[] rotationCurrentAmps = new double[]{};
    }

    public default void updateInputs(RamasseurIOInputs inputs) {}

    /** Assigne une vitesse aux rouleaux du ramasseur */
    public default void setRamasseurVelocityRPM(double rpmTarget) {}

    /** Assigne une position au pivot en radianset précise le slot PID à utiliser */
    public default void setRotationPosition(double targetPosition, int closedLoopSlot) {}

    /** Stop les moteurs du ramasseur */
    public default void stop() {}
}