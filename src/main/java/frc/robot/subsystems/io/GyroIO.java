package frc.robot.subsystems.io;

import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs {
        public boolean connected = false;
        public double yawPositionRad = 0.0;
        public double yawVelocityRadPerSec = 0.0;
    }

    public default void updateInputs(GyroIOInputs inputs) {}
    public default void reset() {}

    public default void addSimulatedAngularVelocity(double radPerSec) {}
}