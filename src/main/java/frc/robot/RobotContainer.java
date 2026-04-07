// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.auto.AutoClimbSequence;
import frc.robot.commands.auto.ShootAllCommand;
import frc.robot.commands.drive.SetBoostModeCommand;
import frc.robot.commands.lanceur.StartFeederCommand;
import frc.robot.commands.lanceur.StopFeederCommand;
import frc.robot.commands.lanceur.ToggleLanceurCommand;
import frc.robot.commands.leds.SetLedsDefault;
import frc.robot.commands.leds.SetLedsRamasser;
import frc.robot.commands.ramasseur.RamasserCommand;
import frc.robot.commands.ramasseur.StopRamasserCommand;
import frc.robot.commands.ramasseur.ToggleSortirRamasseurCommand;
import frc.robot.commands.ramasseur.kickRamasseurCommand;
import frc.robot.commands.ramasseur.kickRamasseurPasSortir;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.LanceurSubsystem;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.RamasseurSubsystem;
import frc.robot.subsystems.io.ClimberIO;
import frc.robot.subsystems.io.ClimberIOReal;
import frc.robot.subsystems.io.ClimberIOSim;
import frc.robot.subsystems.io.GyroIO;
import frc.robot.subsystems.io.GyroIONavX;
import frc.robot.subsystems.io.GyroIOSim;
import frc.robot.subsystems.io.LanceurIO;
import frc.robot.subsystems.io.LanceurIOReal;
import frc.robot.subsystems.io.LanceurIOSim;
import frc.robot.subsystems.io.RamasseurIO;
import frc.robot.subsystems.io.RamasseurIOReal;
import frc.robot.subsystems.io.RamasseurIOSim;
import frc.robot.subsystems.io.SwerveModuleIO;
import frc.robot.subsystems.io.SwerveModuleIOReal;
import frc.robot.subsystems.io.SwerveModuleIOSim;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Conteneur principal du robot.
 *
 * Cette classe déclare les sous-systèmes, la configuration des commandes par défaut
 * et les mappages des boutons. Les commentaires et noms sont en français pour
 * clarifier l'intention.
 */
public class RobotContainer {
    private final SendableChooser<Command> autoChooser;

    // Sous-système de drive
    final DriveSubsystem m_robotDrive;

    // Sous-système de l'accrocheur
    final ClimberSubsystem m_climber;

    // Sous-système des LED
    private final Blinkin m_leds = new Blinkin();

    // Subsystem du lanceur + ramasseur
    final RamasseurSubsystem m_rammasseur;
    private final LanceurSubsystem m_lanceur;

    // Manette du conducteur
    private final XboxController m_driverController =
            new XboxController(OIConstants.kDriverControllerPort);

    // Manette du copilote
    private final XboxController m_copiloteController = 
            new XboxController(OIConstants.kCopiloteControllerPort);

    public RobotContainer() {
        if (RobotBase.isReal()) {
            m_robotDrive = new DriveSubsystem(
                new GyroIONavX(),
                new SwerveModuleIOReal(DriveConstants.kFrontLeftDrivingCanId, DriveConstants.kFrontLeftTurningCanId, DriveConstants.kFrontLeftChassisAngularOffset),
                new SwerveModuleIOReal(DriveConstants.kFrontRightDrivingCanId, DriveConstants.kFrontRightTurningCanId, DriveConstants.kFrontRightChassisAngularOffset),
                new SwerveModuleIOReal(DriveConstants.kRearLeftDrivingCanId, DriveConstants.kRearLeftTurningCanId, DriveConstants.kBackLeftChassisAngularOffset),
                new SwerveModuleIOReal(DriveConstants.kRearRightDrivingCanId, DriveConstants.kRearRightTurningCanId, DriveConstants.kBackRightChassisAngularOffset)
            );
            m_lanceur = new LanceurSubsystem(new LanceurIOReal());
            m_rammasseur = new RamasseurSubsystem(new RamasseurIOReal());
            m_climber = new ClimberSubsystem(new ClimberIOReal());
        } else if (!Constants.kIsReplay) {
            m_robotDrive = new DriveSubsystem(
                new GyroIOSim(),
                new SwerveModuleIOSim(DriveConstants.kFrontLeftChassisAngularOffset),
                new SwerveModuleIOSim(DriveConstants.kFrontRightChassisAngularOffset),
                new SwerveModuleIOSim(DriveConstants.kBackLeftChassisAngularOffset),
                new SwerveModuleIOSim(DriveConstants.kBackRightChassisAngularOffset)
            );
            m_lanceur = new LanceurSubsystem(new LanceurIOSim());
            m_rammasseur = new RamasseurSubsystem(new RamasseurIOSim());
            m_climber = new ClimberSubsystem(new ClimberIOSim());
        } else {
            m_robotDrive = new DriveSubsystem(
                new GyroIO() {},
                new SwerveModuleIO() {},
                new SwerveModuleIO() {},
                new SwerveModuleIO() {},
                new SwerveModuleIO() {}
            );
            m_lanceur = new LanceurSubsystem(new LanceurIO() {});
            m_rammasseur = new RamasseurSubsystem(new RamasseurIO() {});
            m_climber = new ClimberSubsystem(new ClimberIO() {});
        }

        m_leds.set(SparkLedPattern.GREEN); // Couleur par défaut

        NamedCommands.registerCommand("Ramasse", Commands.sequence(new RamasserCommand(m_rammasseur), new SetLedsRamasser(m_leds)));
        NamedCommands.registerCommand("ArreteRamasse", Commands.sequence(new StopRamasserCommand(m_rammasseur), new SetLedsDefault(m_leds)));
        NamedCommands.registerCommand("ToggleLanceur", new ToggleLanceurCommand(m_lanceur));
        NamedCommands.registerCommand("Feed", new StartFeederCommand(m_lanceur));
        NamedCommands.registerCommand("StopFeeder", new StopFeederCommand(m_lanceur));
        NamedCommands.registerCommand("ToggleRamasseur", new ToggleSortirRamasseurCommand(m_rammasseur));
        NamedCommands.registerCommand("kick", new kickRamasseurCommand(m_rammasseur));
        NamedCommands.registerCommand("Vider", new ShootAllCommand(m_lanceur, m_rammasseur));

        configureButtonBindings();
        configureDefaultCommands();

        autoChooser = AutoBuilder.buildAutoChooser();

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    /**
     * Définit la commande par défaut du châssis : lecture des axes de la manette
     * et appel à la méthode "conduire" du sous-système.
     */
    private void configureDefaultCommands() {
        // Lecture des axes, application d'un deadband, et mise à l'échelle par les constantes de vitesse.
        // La méthode conduire(x, y, rot, fieldRelative, autreFlag) est appelée régulièrement.
        m_lanceur.setDefaultCommand(
                new RunCommand(
                        () -> 
                                m_lanceur.startFeeder(m_copiloteController.getRawAxis(3)), 
                        m_lanceur));

        m_robotDrive.setDefaultCommand(
                new RunCommand(
                        () ->
                                m_robotDrive.conduire(
                                        MathUtil.applyDeadband(
                                                m_driverController.getRawAxis(1),
                                                OIConstants.kDriveDeadband),
                                        MathUtil.applyDeadband(
                                                m_driverController.getRawAxis(0),
                                                OIConstants.kDriveDeadband),
                                        -MathUtil.applyDeadband(
                                                m_driverController.getRawAxis(4),
                                                OIConstants.kDriveDeadband),
                                        true,
                                        false),
                        m_robotDrive));
    }

    private void configureButtonBindings() {
        new Trigger(m_copiloteController::getBButton)
                .onTrue(Commands.sequence(new RamasserCommand(m_rammasseur), new SetLedsRamasser(m_leds)))
                .onFalse(Commands.sequence(new StopRamasserCommand(m_rammasseur), new SetLedsDefault(m_leds)));

        new Trigger(m_copiloteController::getYButton)
                .onTrue(new ToggleLanceurCommand(m_lanceur));

        new Trigger(m_copiloteController::getBackButton)
                .onTrue(new ToggleSortirRamasseurCommand(m_rammasseur));

        new Trigger(m_copiloteController::getLeftBumperButton)
                .whileTrue(new kickRamasseurPasSortir(m_rammasseur));

        new Trigger(m_driverController::getXButton)
                .whileTrue(AutoClimbSequence.getClimbSequence(m_robotDrive));

        new Trigger(m_driverController::getRightBumperButton)
                .onTrue(new SetBoostModeCommand(m_robotDrive, true))
                .onFalse(new SetBoostModeCommand(m_robotDrive, false));

        // new Trigger(m_driverController::getXButton)
        //         .whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));

        new Trigger(m_driverController::getLeftBumperButton)
                .whileTrue(
                        new RunCommand(
                                () -> {

                                    Boolean isOnGoodSide = m_robotDrive.isOnGoodSide();

                                    if (!isOnGoodSide) {
                                        m_robotDrive.conduire(
                                                MathUtil.applyDeadband(
                                                        m_driverController.getRawAxis(1),
                                                        OIConstants.kDriveDeadband),
                                                MathUtil.applyDeadband(
                                                        m_driverController.getRawAxis(0),
                                                        OIConstants.kDriveDeadband),
                                                -MathUtil.applyDeadband(
                                                        -m_driverController.getRawAxis(4),
                                                        OIConstants.kDriveDeadband),
                                                true,
                                                false);
                                        return;
                                    }

                                    m_leds.set(SparkLedPattern.RED); // Couleur de tracking

                                    // Vitesses demandées (field-relative)
                                    double vx =
                                            -MathUtil.applyDeadband(
                                                    m_driverController.getRawAxis(1),
                                                    OIConstants.kDriveDeadband);
                                    double vy =
                                            -MathUtil.applyDeadband(
                                                    m_driverController.getRawAxis(0),
                                                    OIConstants.kDriveDeadband);

                                    // Centre du panier selon l'alliance
                                    double basketY = AutoConstants.kBasketY;
                                    double basketX =
                                            m_robotDrive.isRedAlliance()
                                                    ? AutoConstants.kBasketXRed
                                                    : AutoConstants.kBasketXBlue;

                                    double px = m_robotDrive.getPose().getX();
                                    double py = m_robotDrive.getPose().getY();

                                    double dx = px - basketX;
                                    double dy = py - basketY;
                                    double dist = Math.hypot(dx, dy);
                                    if (dist < 1e-3) dist = 1e-3; // éviter division par zéro

                                    double nx = dx / dist;
                                    double ny = dy / dist;

                                    double radial = vx * nx + vy * ny;
                                    double vxT = vx - radial * nx;
                                    double vyT = vy - radial * ny;

                                    // Correction
                                    double radiusError = dist - 2.4;
                                    double kR = 0.95; // gain radial
                                    double radialCorr = -kR * radiusError;

                                    SmartDashboard.putNumber("Radius Error", radiusError);

                                    double vxCmd = vxT + radialCorr * nx;
                                    double vyCmd = vyT + radialCorr * ny;

                                    // Clamp de sécurité
                                    vxCmd =
                                            MathUtil.clamp(
                                                    vxCmd, -0.85, 0.85);
                                    vyCmd =
                                            MathUtil.clamp(
                                                    vyCmd, -0.85, 0.85);

                                    SmartDashboard.putNumber("Angle to Basket", m_robotDrive.getAngleToBasket().getDegrees());

                                    double rotCmd =
                                            MathUtil.applyDeadband(
                                                    m_robotDrive.getCompensationRotation(m_robotDrive.getAngleToBasket().getDegrees()),
                                                    0.01);

                                    m_robotDrive.conduire(vxCmd, vyCmd, rotCmd, true, false);
                                },
                                m_robotDrive))
                .onFalse(new SetLedsDefault(m_leds));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
