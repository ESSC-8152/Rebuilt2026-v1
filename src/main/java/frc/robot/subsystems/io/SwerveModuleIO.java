package frc.robot.subsystems.io;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface SwerveModuleIO {
    @AutoLog
    public static class SwerveModuleIOInputs {
        public double drivePositionRad = 0.0;
        public double driveVelocityRadPerSec = 0.0;
        public double turnAbsolutePositionRad = 0.0;
        public double turnVelocityRadPerSec = 0.0;
    }

    public default void updateInputs(SwerveModuleIOInputs inputs) {}
    public default void setDesiredState(SwerveModuleState desiredState) {}
    public default void resetEncoders() {}
}