package frc.robot.subsystems;

import frc.robot.Constants.RamasseurConstants;
import frc.robot.configs.RamasseurConfigs;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RamasseurSubsystem extends SubsystemBase {
    private final SparkFlex moteurRamasseur;
    private final SparkMax moteurRotationRamasseur;

    private final SparkClosedLoopController ramasseurPidController;
    private final SparkClosedLoopController rotationRamasseurPidController;

    public boolean isRamasseurDeployed = false;
    private double targetPosition = 0;

    public RamasseurSubsystem() {
        moteurRamasseur = new SparkFlex(RamasseurConstants.kMoteurRamasseurID, MotorType.kBrushless);
        moteurRotationRamasseur = new SparkMax(RamasseurConstants.kMoteurRotationRamasseurID, MotorType.kBrushless);

        ramasseurPidController = moteurRamasseur.getClosedLoopController();
        rotationRamasseurPidController = moteurRotationRamasseur.getClosedLoopController();

        moteurRamasseur.configure(RamasseurConfigs.RamasseurSubsystem.ramasseurConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        moteurRotationRamasseur.configure(RamasseurConfigs.RamasseurSubsystem.rotationRamasseurConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void ramasser() {
        ramasseurPidController.setSetpoint(RamasseurConstants.kVitesseRamasseur, ControlType.kVelocity);
    }

    public void rentrerRamasseur() {
        isRamasseurDeployed = false;
        targetPosition = RamasseurConstants.kRetractedPosition;
        rotationRamasseurPidController.setSetpoint(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    public void sortirRamasseur() {
        isRamasseurDeployed = true;
        targetPosition = RamasseurConstants.kExtendedPosition;
        rotationRamasseurPidController.setSetpoint(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    public void kickRamasseur() {
        if (!isRamasseurDeployed) {
            return;
        }

        targetPosition = RamasseurConstants.kMidPosition;
        rotationRamasseurPidController.setSetpoint(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot1);
    }

    public boolean isAtPosition() {
        double current = moteurRotationRamasseur.getAbsoluteEncoder().getPosition();

        return Math.abs(targetPosition - current) < 0.1;
    }

    public void stop() {
        moteurRamasseur.stopMotor();
    }
}