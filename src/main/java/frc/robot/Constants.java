// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    public static final double kMaxSpeedMetersPerSecond = 2;
    public static final double kMaxAngularSpeed = Math.PI; // radians per second

    public static final PIDController xController = new PIDController(0.5, 0.0, 0.0);
    public static final PIDController yController = new PIDController(0.5, 0.0, 0.0);
    public static final PIDController thetaController = new PIDController(0.67, 0.0, 0.05);

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(25);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(23);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2; //0.1307733
    public static final double kFrontRightChassisAngularOffset = 0 ; // 0.3512427
    public static final double kBackLeftChassisAngularOffset = Math.PI; // 0.0146979
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;  //0.4225199

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 3;
    public static final int kRearLeftDrivingCanId = 5;
    public static final int kFrontRightDrivingCanId = 1;
    public static final int kRearRightDrivingCanId = 7;

    public static final int kFrontLeftTurningCanId = 4;
    public static final int kRearLeftTurningCanId = 6;
    public static final int kFrontRightTurningCanId = 2;
    public static final int kRearRightTurningCanId = 8;

    public static final boolean kGyroReversed = true;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 13;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters =  Units.inchesToMeters(3);
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 40; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kCopiloteControllerPort = 1;
    public static final double kDriveDeadband = 0.05;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 1;
    public static final double kMaxAccelerationMetersPerSecondSquared = 0.25;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1.0;
    public static final double kPYController = 1.0;
    public static final double kPThetaController = 1.0;

    public static final double kBasketXRed = 11.9154;
    public static final double kBasketXBlue = 4.6256;
    public static final double kBasketY = 4.0;

    public static final Pose2d kClimbPose = new Pose2d(15.572, 3.430, new Rotation2d(Math.PI));

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class VortexMotorConstants {
    public static final double kVortexMotorFreeSpeedRpm = 6784;
  }

  public static final class LedConstants {
    // Port du REVBlinkinLedController
    public static final int kBlinkinPwmPort = 0;
  }

  public static final class LanceurConstants {
    // ID des moteurs
    public static final int kMoteurGaucheLanceurID = 11;
    public static final int kMoteurDroitLanceurID = 12;
    public static final int kMoteurFeederID = 13;

    // Vitesses (rpm)
    public static final int kVitesseLanceurLent = 500;
    public static final int kVitesseLanceur = 3000;
    public static final int kVitesseLanceurVite = 5000;
    public static final int kVitesseFeeder = 5000;
    public static final double kVitesseCourroies = 3500;
    public static final int kMoteurFeederBaseID = 14;
  }

  public static final class RamasseurConstants {
    // ID des moteurs
    public static final int kMoteurRamasseurID = 9;
    public static final int kMoteurRotationRamasseurID = 10;

    // PID Ajuster le P pour la vitesse de rétraction/extension
    public static final double kRotationP = 0.6;
    public static final double kRotationI = 0.0;
    public static final double kRotationD = 0.0;

    // Limite de courant (Amp)
    public static final int kRotationCurrentLimit = 30;

    // Vitesse de rotation du ramasseur (rpm)
    public static final int kVitesseRamasseur = 5250;

    // Position setpoints de l'encoder (radians)
    public static final double kExtendedPosition = 1.9;
    public static final double kRetractedPosition = 0;
    public static final double kMidPosition = Math.PI/3;
  }

  public static final class ClimberConstants {
    // ID des moteurs
    public static final int kMoteurClimberID = 16;

    //Positions du climber
    public static final double kClimberUpPosition = 16; // Ajuster
    public static final double kClimberAccrochePosition = 0.25;
    public static final double kClimberDownPosition = 0;

    public static int kHomeSwitchPort = 0; 
  }

  public static final boolean kIsReplay = false; // Set to true to enable replay mode (requires log files in "logs" directory)
}

