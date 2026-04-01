package frc.robot.subsystems.io;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs {
        public double climberPositionRad = 0.0;
        public double climberVelocityRadPerSec = 0.0;
    }

    public default void updateInputs(ClimberIOInputs inputs) {}
    public default void setClimberPositionRad(double positionRad) {}
}
