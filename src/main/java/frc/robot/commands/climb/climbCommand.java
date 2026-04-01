package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.ClimberSubsystem;

public class climbCommand extends Command{
    private final ClimberSubsystem m_climb;

    public climbCommand(ClimberSubsystem climb) {
        m_climb = climb;
    }

    @Override
    public void initialize() {
        m_climb.setClimberPosition(ClimberConstants.kClimberUpPosition);        
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
        m_climb.setClimberPosition(ClimberConstants.kClimberDownPosition);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
