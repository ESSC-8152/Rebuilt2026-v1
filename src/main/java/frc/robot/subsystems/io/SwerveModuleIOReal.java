package frc.robot.subsystems.io;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.configs.DriveConfigs;

public class SwerveModuleIOReal implements SwerveModuleIO {
    private final SparkFlex m_drivingSpark;
    private final SparkMax m_turningSpark;
    private final RelativeEncoder m_drivingEncoder;
    private final AbsoluteEncoder m_turningEncoder;
    private final SparkClosedLoopController m_drivingClosedLoopController;
    private final SparkClosedLoopController m_turningClosedLoopController;
    private final double m_chassisAngularOffset;

    public SwerveModuleIOReal(int drivingCANId, int turningCANId, double chassisAngularOffset) {
        m_drivingSpark = new SparkFlex(drivingCANId, MotorType.kBrushless);
        m_turningSpark = new SparkMax(turningCANId, MotorType.kBrushless);

        m_drivingEncoder = m_drivingSpark.getEncoder();
        m_turningEncoder = m_turningSpark.getAbsoluteEncoder();

        m_drivingClosedLoopController = m_drivingSpark.getClosedLoopController();
        m_turningClosedLoopController = m_turningSpark.getClosedLoopController();

        m_drivingSpark.configure(DriveConfigs.MAXSwerveModule.drivingConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_turningSpark.configure(DriveConfigs.MAXSwerveModule.turningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_chassisAngularOffset = chassisAngularOffset;
        m_drivingEncoder.setPosition(0);
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        inputs.drivePositionRad = m_drivingEncoder.getPosition();
        inputs.driveVelocityRadPerSec = m_drivingEncoder.getVelocity();
        inputs.turnAbsolutePositionRad = m_turningEncoder.getPosition();
        inputs.turnVelocityRadPerSec = m_turningEncoder.getVelocity();
    }

    @Override
    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

        correctedDesiredState.optimize(new Rotation2d(m_turningEncoder.getPosition()));

        m_drivingClosedLoopController.setSetpoint(correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
        m_turningClosedLoopController.setSetpoint(correctedDesiredState.angle.getRadians(), ControlType.kPosition);
    }

    @Override
    public void resetEncoders() {
        m_drivingEncoder.setPosition(0);
    }
}