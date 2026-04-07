package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ForceMonterClimberCommand extends Command{
    private final ClimberSubsystem m_climb;

    public ForceMonterClimberCommand(ClimberSubsystem climb) {
        m_climb = climb;

        addRequirements(climb);
    }

    @Override
    public void initialize() {
        m_climb.setSpeed(0.2);
    }

    @Override
    public void end(boolean interrupted) {
        m_climb.setSpeed(0.0);
    }
}
