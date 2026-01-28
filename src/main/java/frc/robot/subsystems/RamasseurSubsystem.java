package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RamasseurSubsystem extends SubsystemBase{
    private final SparkFlex moteurRamasseur;

    private final SparkClosedLoopController ramasseurPidController;

    private final SparkFlexConfig ramasseurConfig;

    public RamasseurSubsystem(){
        moteurRamasseur = new SparkFlex(9, MotorType.kBrushless);

        ramasseurPidController = moteurRamasseur.getClosedLoopController();

        ramasseurConfig = new SparkFlexConfig();

        ramasseurConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(0.0001)
            .i(0.0)
            .d(0.0)
            .outputRange(-1, 1)
            .feedForward.kV(0.00016);

        ramasseurConfig.signals
            .primaryEncoderVelocityPeriodMs(20);

        moteurRamasseur.configure(ramasseurConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void ramasser(){
        ramasseurPidController.setSetpoint(-1000, com.revrobotics.spark.SparkBase.ControlType.kVelocity);
    }

    public void stop(){
        moteurRamasseur.set(0);
    }
}