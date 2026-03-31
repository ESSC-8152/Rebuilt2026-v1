package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.io.SwerveModuleIO;
import frc.robot.subsystems.io.SwerveModuleIOInputsAutoLogged;

public class MAXSwerveModule {
    private final SwerveModuleIO io;
    private final SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();
    private final String name;
    private final double m_chassisAngularOffset;

    public MAXSwerveModule(String name, SwerveModuleIO io, double chassisAngularOffset) {
        this.name = name;
        this.io = io;
        this.m_chassisAngularOffset = chassisAngularOffset;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive/" + name, inputs);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            inputs.driveVelocityRadPerSec,
            new Rotation2d(inputs.turnAbsolutePositionRad - m_chassisAngularOffset)
        );
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            inputs.drivePositionRad,
            new Rotation2d(inputs.turnAbsolutePositionRad - m_chassisAngularOffset)
        );
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        io.setDesiredState(desiredState);
    }

    public void resetEncoders() {
        io.resetEncoders();
    }
}