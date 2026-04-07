package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class AlignToPosePIDCommand extends Command {
    private final DriveSubsystem m_robotDrive;
    private final Pose2d targetPose;

    private final PIDController xController;
    private final PIDController yController;
    private final PIDController thetaController;

    private static final double X_TOLERANCE_METERS = 0.05;
    private static final double Y_TOLERANCE_METERS = 0.01;
    private static final double THETA_TOLERANCE_RADIANS = Units.degreesToRadians(2);

    public AlignToPosePIDCommand(DriveSubsystem m_robotDrive, Pose2d targetPose) {
        this.m_robotDrive = m_robotDrive;
        this.targetPose = targetPose;

        this.xController = DriveConstants.xController;
        this.yController = DriveConstants.yController;
        this.thetaController = DriveConstants.thetaController;

        this.thetaController.enableContinuousInput(-Math.PI, Math.PI);

        this.xController.setTolerance(X_TOLERANCE_METERS);
        this.yController.setTolerance(Y_TOLERANCE_METERS);
        this.thetaController.setTolerance(THETA_TOLERANCE_RADIANS);

        addRequirements(m_robotDrive);
    }

    @Override
    public void initialize() {
        xController.reset();
        yController.reset();
        thetaController.reset();
    }

    @Override
    public void execute() {
        Pose2d currentPose = m_robotDrive.getPose();

        double xCmd = xController.calculate(currentPose.getX(), targetPose.getX());
        double yCmd = yController.calculate(currentPose.getY(), targetPose.getY());
        double rotCmd = thetaController.calculate(
            currentPose.getRotation().getRadians(), 
            targetPose.getRotation().getRadians()
        );

        m_robotDrive.conduire(xCmd, yCmd, rotCmd, true, false);
    }

    @Override
    public boolean isFinished() {
        return xController.atSetpoint() 
            && yController.atSetpoint() 
            && thetaController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        m_robotDrive.stop();
    }
}