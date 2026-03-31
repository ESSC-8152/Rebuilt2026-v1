package frc.robot.subsystems.io;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants.DriveConstants;

public class GyroIONavX implements GyroIO {
    private AHRS m_realGyro;

    public GyroIONavX() {
        if (RobotBase.isReal()) {
            try {
                m_realGyro = new AHRS(NavXComType.kMXP_SPI);
            } catch (RuntimeException ex) {}
        }
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        if (m_realGyro != null) {
            inputs.connected = m_realGyro.isConnected();
            double yaw = m_realGyro.getYaw();
            double finalYaw = DriveConstants.kGyroReversed ? -yaw : yaw;
            
            inputs.yawPositionRad = Units.degreesToRadians(finalYaw);
            inputs.yawVelocityRadPerSec = Units.degreesToRadians(m_realGyro.getRate());
        } else {
            inputs.connected = false;
        }
    }

    @Override
    public void reset() {
        if (m_realGyro != null) {
            m_realGyro.reset();
        }
    }
}