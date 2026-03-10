package frc.robot.subsystems;

import frc.robot.Constants.RamasseurConstants;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RamasseurSubsystem extends SubsystemBase{
    private final SparkFlex moteurRamasseur;
    private final SparkMax moteurRotationRamasseur;

    private final SparkClosedLoopController ramasseurPidController;
    private final SparkClosedLoopController rotationRamasseurPidController;

    private final SparkFlexConfig ramasseurConfig;
    private final SparkMaxConfig rotationRamasseurConfig;

    public boolean isRamasseurDeployed = false;
    private double targetPosition = 0;

    public RamasseurSubsystem(){
        moteurRamasseur = new SparkFlex(RamasseurConstants.kMoteurRamasseurID, MotorType.kBrushless);
        moteurRotationRamasseur = new SparkMax(RamasseurConstants.kMoteurRotationRamasseurID, MotorType.kBrushless);

        ramasseurPidController = moteurRamasseur.getClosedLoopController();
        rotationRamasseurPidController = moteurRotationRamasseur.getClosedLoopController();

        ramasseurConfig = new SparkFlexConfig();
        rotationRamasseurConfig = new SparkMaxConfig();

        double turningFactor = 2 * Math.PI;

        ramasseurConfig
            .smartCurrentLimit(40)
            .idleMode(IdleMode.kBrake)
            .inverted(true);

        ramasseurConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            // Ajoutez un petit gain P (à tester et ajuster progressivement)
            .p(0.0008) 
            .i(0.0)
            .d(0.0)
            .outputRange(-1, 1)
            .feedForward.kV(0.00016);

        ramasseurConfig.signals
            .primaryEncoderVelocityPeriodMs(20);

    rotationRamasseurConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(RamasseurConstants.kRotationCurrentLimit)
            .openLoopRampRate(2)
            .closedLoopRampRate(2);

    rotationRamasseurConfig.absoluteEncoder
            .positionConversionFactor(turningFactor) // radians
            .velocityConversionFactor(turningFactor / 60.0); // radians par seconde

        rotationRamasseurConfig
            .openLoopRampRate(0.0)
            .closedLoopRampRate(0.0); 

        rotationRamasseurConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(0, turningFactor)

            // --- SLOT 0 : Mouvement normal
            .pid(RamasseurConstants.kRotationP, RamasseurConstants.kRotationI, RamasseurConstants.kRotationD, ClosedLoopSlot.kSlot0)
            .outputRange(-0.6, 0.6, ClosedLoopSlot.kSlot0)

            // --- SLOT 1 : Coup
            .pid(0.8, 0, 0.6, ClosedLoopSlot.kSlot1) 
            .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

        moteurRamasseur.configure(ramasseurConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        moteurRotationRamasseur.configure(rotationRamasseurConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void ramasser(){
        ramasseurPidController.setSetpoint(RamasseurConstants.kVitesseRamasseur, ControlType.kVelocity);
    }

    public void rentrerRamasseur(){
        isRamasseurDeployed = false;
        targetPosition = RamasseurConstants.kRetractedPosition;
        rotationRamasseurPidController.setSetpoint(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    public void sortirRamasseur(){
        isRamasseurDeployed = true;
        targetPosition = RamasseurConstants.kExtendedPosition;
        rotationRamasseurPidController.setSetpoint(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }
    
    public void kickRamasseur(){
        if (!isRamasseurDeployed) return;

        targetPosition = RamasseurConstants.kMidPosition;
        rotationRamasseurPidController.setSetpoint(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot1);
    }

    public boolean isAtPosition(){
        double current = moteurRotationRamasseur.getAbsoluteEncoder().getPosition();

        SmartDashboard.putNumber("Encoder position",current);
        SmartDashboard.putNumber("Target", targetPosition);

        return Math.abs(targetPosition - current) < 0.1 ? true : false;
    }

    public void stop(){
        moteurRamasseur.set(0);
    }
}