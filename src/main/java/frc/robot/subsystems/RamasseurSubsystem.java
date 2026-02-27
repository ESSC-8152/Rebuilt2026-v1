package frc.robot.subsystems;

import frc.robot.Constants.RamasseurConstants;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RamasseurSubsystem extends SubsystemBase{
    private final SparkFlex moteurRamasseur;
    private final SparkMax moteurRotationRamasseur;

    private final SparkClosedLoopController ramasseurPidController;
    private final SparkClosedLoopController rotationRamasseurPidController;

    private final SparkFlexConfig ramasseurConfig;
    private final SparkMaxConfig rotationRamasseurConfig;

    public boolean isRamasseurDeployed = false;

    public RamasseurSubsystem(){
        moteurRamasseur = new SparkFlex(RamasseurConstants.kMoteurRamasseurID, MotorType.kBrushless);
        moteurRotationRamasseur = new SparkMax(RamasseurConstants.kMoteurRotationRamasseurID, MotorType.kBrushless);

        ramasseurPidController = moteurRamasseur.getClosedLoopController();
        rotationRamasseurPidController = moteurRotationRamasseur.getClosedLoopController();

        ramasseurConfig = new SparkFlexConfig();
        rotationRamasseurConfig = new SparkMaxConfig();

        double turningFactor = 2 * Math.PI;

        ramasseurConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(0.0)
            .i(0.0)
            .d(0.0)
            .outputRange(-1, 1)
            .feedForward.kV(0.00016);

        ramasseurConfig.signals
            .primaryEncoderVelocityPeriodMs(20);

    rotationRamasseurConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(RamasseurConstants.kRotationCurrentLimit);
        rotationRamasseurConfig.absoluteEncoder
                .positionConversionFactor(turningFactor) // radians
                .velocityConversionFactor(turningFactor / 60.0); // radians per second
    rotationRamasseurConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)

        .pid(RamasseurConstants.kRotationP, RamasseurConstants.kRotationI, RamasseurConstants.kRotationD)
        .outputRange(-1, 1)
                // Enable PID wrap around for the turning motor. This will allow the PID
                // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
                // to 10 degrees will go through 0 rather than the other direction which is a
                // longer route.
                .positionWrappingEnabled(true)
                .positionWrappingInputRange(0, turningFactor);

        moteurRamasseur.configure(ramasseurConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        moteurRotationRamasseur.configure(rotationRamasseurConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void ramasser(){
        ramasseurPidController.setSetpoint(RamasseurConstants.kVitesseRamasseur, ControlType.kVelocity);
    }

    public void rentrerRamasseur(){
        isRamasseurDeployed = false;
        rotationRamasseurPidController.setSetpoint(RamasseurConstants.kRetractedPosition, ControlType.kPosition);
    }

    public void sortirRamasseur(){
        isRamasseurDeployed = true;
        rotationRamasseurPidController.setSetpoint(RamasseurConstants.kExtendedPosition, ControlType.kPosition);
    }

    public void stop(){
        moteurRamasseur.stopMotor();
    }
}