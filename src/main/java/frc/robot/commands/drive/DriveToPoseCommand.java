package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.AutoConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

public final class DriveToPoseCommand {
    private DriveToPoseCommand() {}

    public static Command create(Pose2d targetPose) {

        return Commands.deferredProxy(() -> {
                return AutoBuilder.pathfindToPose(
                    targetPose,
                    new PathConstraints(
                        AutoConstants.kMaxSpeedMetersPerSecond,
                        AutoConstants.kMaxAccelerationMetersPerSecondSquared,
                        Units.degreesToRadians(90),
                        Units.degreesToRadians(180)
                    ),
                    0.0
                );
        });
    }
}