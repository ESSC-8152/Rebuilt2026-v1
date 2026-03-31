package frc.robot.subsystems.io;

import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModuleIOSim implements SwerveModuleIO {
    private double drivePositionMeters = 0.0;
    private double driveVelocityMetersPerSec = 0.0;
    private double turnAbsolutePositionRad = 0.0;
    
    private final double m_chassisAngularOffset;

    public SwerveModuleIOSim(double chassisAngularOffset) {
        this.m_chassisAngularOffset = chassisAngularOffset;
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        drivePositionMeters += driveVelocityMetersPerSec * 0.02;

        inputs.drivePositionRad = drivePositionMeters;
        inputs.driveVelocityRadPerSec = driveVelocityMetersPerSec;
        inputs.turnAbsolutePositionRad = turnAbsolutePositionRad;
        inputs.turnVelocityRadPerSec = 0.0;
    }

    @Override
    public void setDesiredState(SwerveModuleState desiredState) {
        driveVelocityMetersPerSec = desiredState.speedMetersPerSecond;
        turnAbsolutePositionRad = desiredState.angle.getRadians() + m_chassisAngularOffset;
    }

    @Override
    public void resetEncoders() {
        drivePositionMeters = 0.0;
    }
}