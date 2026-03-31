package frc.robot.subsystems.io;

public class GyroIOSim implements GyroIO {
    private double yawPositionRad = 0.0;
    private double yawVelocityRadPerSec = 0.0;

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = true;
        
        yawPositionRad += yawVelocityRadPerSec * 0.02;

        inputs.yawPositionRad = yawPositionRad;
        inputs.yawVelocityRadPerSec = yawVelocityRadPerSec;
    
        yawVelocityRadPerSec = 0.0;
    }

    @Override
    public void addSimulatedAngularVelocity(double radPerSec) {
        this.yawVelocityRadPerSec = radPerSec;
    }
}