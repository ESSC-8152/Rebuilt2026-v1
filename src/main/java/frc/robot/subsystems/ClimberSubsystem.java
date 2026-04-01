package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.io.ClimberIO;
import frc.robot.subsystems.io.ClimberIOInputsAutoLogged;

public class ClimberSubsystem extends SubsystemBase {
     private ClimberIO io;
     private ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

     public ClimberSubsystem(ClimberIO climberIO) {
         this.io = climberIO;
     }

     @Override
     public void periodic() {
         io.updateInputs(inputs);
         Logger.processInputs("Climber", inputs);
     }

     public void setClimberPosition(double positionRad) {
         io.setClimberPositionRad(positionRad);
     }
    
}
