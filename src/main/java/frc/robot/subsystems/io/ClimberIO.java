package frc.robot.subsystems.io;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs {
        public double climberPositionRad = 0.0;
        public double climberVelocityRadPerSec = 0.0;
        public boolean climberHomeSwitch = false;
        public double[] climberCurrentAmps = new double[]{};
    }

    public default void updateInputs(ClimberIOInputs inputs) {}
    public default void monterClimberPositionRad(double positionRad) {}
    public default void setTargetPositionRad(double positionRad, int slot, double feedforwardVolts) {}
    public default void resetEncoder(double positionRad) {}
    public default void stop() {}
    public default void setSpeed(double speed) {}
}
