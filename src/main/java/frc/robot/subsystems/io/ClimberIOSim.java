package frc.robot.subsystems.io;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.ClimberConstants;

public class ClimberIOSim implements ClimberIO {
    private static final double kMaxVelocityRadPerSec = Math.PI;
    private static final double kMaxAccelerationRadPerSecSq = 2 * Math.PI;
    private static final double kPositionKp = 0.6;

    private double climberPositionRad = 0.0;
    private double climberVelocityRadPerSec = 0.0;
    private double targetPositionRad = 0.0;
    private double lastUpdateTimeSec = Timer.getFPGATimestamp();

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        double now = Timer.getFPGATimestamp();
        double dt = now - lastUpdateTimeSec;
        if (dt <= 0.0 || Double.isNaN(dt) || Double.isInfinite(dt)) {
            dt = 0.02;
        }

        double error = targetPositionRad - climberPositionRad;
        double desiredVelocity = MathUtil.clamp(error * kPositionKp, -kMaxVelocityRadPerSec, kMaxVelocityRadPerSec);
        double maxDeltaVel = kMaxAccelerationRadPerSecSq * dt;
        climberVelocityRadPerSec += MathUtil.clamp(
                desiredVelocity - climberVelocityRadPerSec, -maxDeltaVel, maxDeltaVel);

        climberPositionRad += climberVelocityRadPerSec * dt;
        double minPosition = Math.min(ClimberConstants.kClimberDownPosition, ClimberConstants.kClimberUpPosition);
        double maxPosition = Math.max(ClimberConstants.kClimberDownPosition, ClimberConstants.kClimberUpPosition);
        climberPositionRad = MathUtil.clamp(climberPositionRad, minPosition, maxPosition);
        if ((climberPositionRad == minPosition && climberVelocityRadPerSec < 0.0)
                || (climberPositionRad == maxPosition && climberVelocityRadPerSec > 0.0)) {
            climberVelocityRadPerSec = 0.0;
        }
        lastUpdateTimeSec = now;

        inputs.climberPositionRad = climberPositionRad;
        inputs.climberVelocityRadPerSec = climberVelocityRadPerSec;
    }

    @Override
    public void setClimberPositionRad(double positionRad) {
        double minPosition = Math.min(ClimberConstants.kClimberDownPosition, ClimberConstants.kClimberUpPosition);
        double maxPosition = Math.max(ClimberConstants.kClimberDownPosition, ClimberConstants.kClimberUpPosition);
        targetPositionRad = MathUtil.clamp(positionRad, minPosition, maxPosition);
    }
}
