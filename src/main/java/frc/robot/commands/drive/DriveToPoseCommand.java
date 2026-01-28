package frc.robot.commands.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.AutoConstants;

/**
 * Utility to create a pathfinding command to a target pose using AutoBuilder.
 *
 * Usage:
 *   Command cmd = DriveToPoseCommand.create(targetPose);
 */
public final class DriveToPoseCommand {
    private DriveToPoseCommand() {}

    public static Command create(Pose2d targetPose) {
        return Commands.deferredProxy(() ->
            AutoBuilder.pathfindToPose(
                targetPose,
                new PathConstraints(
                    AutoConstants.kMaxSpeedMetersPerSecond,
                    AutoConstants.kMaxAccelerationMetersPerSecondSquared,
                    Units.degreesToRadians(540),
                    Units.degreesToRadians(720)
                ),
                0.0
            )
        );
    }
}