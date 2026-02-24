// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.TeleopConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LanceurSubsystem;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.RamasseurSubsystem;
import frc.robot.commands.drive.DriveToPoseCommand;
import frc.robot.commands.lanceur.StopLanceurCommand;
import frc.robot.commands.lanceur.StartLanceurCommand;
import frc.robot.commands.leds.SetLedsDefault;
import frc.robot.commands.leds.SetLedsRamasser;
import frc.robot.commands.ramasseur.RamasserCommand;
import frc.robot.commands.ramasseur.ToggleSortirRamasseurCommand;
import frc.robot.commands.ramasseur.StopRamasserCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
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

    // Sous-système du châssis (drive)
    final DriveSubsystem m_robotDrive = new DriveSubsystem();

    // Sous-système des LEDs
    private final Blinkin m_leds = new Blinkin();

    // Subsystem du lanceur + ramasseur
    final RamasseurSubsystem m_rammasseur = new RamasseurSubsystem();
    private final LanceurSubsystem m_lanceur = new LanceurSubsystem();

    // Manette du conducteur
    private final XboxController m_driverController =
            new XboxController(OIConstants.kDriverControllerPort);

    /**
     * Constructeur : configure les bindings et la commande par défaut.
     */
    public RobotContainer() {
        m_leds.set(SparkLedPattern.GREEN); // Couleur par défaut

        NamedCommands.registerCommand("Ramasse", Commands.sequence(new RamasserCommand(m_rammasseur), new SetLedsRamasser(m_leds)));
        NamedCommands.registerCommand("ArreteRamasse", Commands.sequence(new StopRamasserCommand(m_rammasseur), new SetLedsDefault(m_leds)));

        configureButtonBindings();
        configureDefaultCommands();

        autoChooser = AutoBuilder.buildAutoChooser();

        SmartDashboard.putData("Auto Chooser", autoChooser);

                // Do NOT reset the alliance starting pose here; the Driver Station may not
                // be connected at constructor time. DriveSubsystem will wait and apply the
                // alliance-based starting pose once the DS/FMS reports the alliance.
    }

    /**
     * Définit la commande par défaut du châssis : lecture des axes de la manette
     * et appel à la méthode "conduire" du sous-système.
     */
    private void configureDefaultCommands() {
        // Lecture des axes, application d'un deadband, et mise à l'échelle par les constantes de vitesse.
        // La méthode conduire(x, y, rot, fieldRelative, autreFlag) est appelée régulièrement.
        m_robotDrive.setDefaultCommand(
                new RunCommand(
                        () ->
                                m_robotDrive.conduire(
                                        -MathUtil.applyDeadband(
                                                m_driverController.getRawAxis(1) * DriveConstants.kVitesse,
                                                OIConstants.kDriveDeadband),
                                        -MathUtil.applyDeadband(
                                                m_driverController.getRawAxis(0) * DriveConstants.kVitesse,
                                                OIConstants.kDriveDeadband),
                                        -MathUtil.applyDeadband(
                                                m_driverController.getRawAxis(4) * DriveConstants.kVitesseRotation,
                                                OIConstants.kDriveDeadband),
                                        true, // fieldRelative : true si contrôle relatif au terrain
                                        false),
                        m_robotDrive));
    }

    /**
     * Configure les mappages boutons -> commandes.
     * - Bouton 8 : remettre l'odométrie / position zéro du châssis.
     * - Bouton 1 : lancer la commande de déplacement vers une Pose fixe.
     */
    private void configureButtonBindings() {
        // Quand on appuie (onTrue) sur le bouton 8, exécuter instantanément resetToAllianceStartingPose()
        new JoystickButton(m_driverController, 8)
                .onTrue(new InstantCommand(m_robotDrive::resetToAllianceStartingPose, m_robotDrive));

        new JoystickButton(m_driverController, 1)
                .onTrue(Commands.sequence(new RamasserCommand(m_rammasseur), new SetLedsRamasser(m_leds)))
                .onFalse(Commands.sequence(new StopRamasserCommand(m_rammasseur), new SetLedsDefault(m_leds)));

        new JoystickButton(m_driverController, 2)
                .onTrue(new ToggleSortirRamasseurCommand(m_rammasseur));

        new JoystickButton(m_driverController, 3)
                .onTrue(new StartLanceurCommand(m_lanceur))
                .onFalse(new StopLanceurCommand(m_lanceur));

        new Trigger(m_driverController::getRightBumperButton)
                .onTrue(DriveToPoseCommand.create(new Pose2d(7.7877356522024925, 5.115507523646952, new Rotation2d(Math.toRadians(25.559982054158734)))));

        new Trigger(m_driverController::getLeftBumperButton)
                .whileTrue(
                        new RunCommand(
                                () -> {
                                    m_leds.set(SparkLedPattern.RED); // Couleur de tracking

                                    // Vitesses demandées (field-relative)
                                    double vx =
                                            -MathUtil.applyDeadband(
                                                    m_driverController.getRawAxis(1) * DriveConstants.kVitesse,
                                                    OIConstants.kDriveDeadband);
                                    double vy =
                                            -MathUtil.applyDeadband(
                                                    m_driverController.getRawAxis(0) * DriveConstants.kVitesse,
                                                    OIConstants.kDriveDeadband);

                                    // Centre du panier selon l'alliance
                                    double basketY = TeleopConstants.kBasketY;
                                    double basketX =
                                            m_robotDrive.isRedAlliance()
                                                    ? TeleopConstants.kBasketXRed
                                                    : TeleopConstants.kBasketXBlue;

                                    double px = m_robotDrive.getPose().getX();
                                    double py = m_robotDrive.getPose().getY();

                                    double dx = px - basketX;
                                    double dy = py - basketY;
                                    double dist = Math.hypot(dx, dy);
                                    if (dist < 1e-3) dist = 1e-3; // éviter division par zéro

                                    double nx = dx / dist;
                                    double ny = dy / dist;

                                    // Projection tangentielle : retirer la composante radiale
                                    double radial = vx * nx + vy * ny;
                                    double vxT = vx - radial * nx;
                                    double vyT = vy - radial * ny;

                                    // Correction douce pour rester à rayon 1.5 m
                                    double radiusError = dist - 1.5;
                                    double kR = 0.95; // gain radial
                                    double radialCorr = -kR * radiusError;

                                    SmartDashboard.putNumber("Radius Error", radiusError);

                                    double vxCmd = vxT + radialCorr * nx;
                                    double vyCmd = vyT + radialCorr * ny;

                                    // Clamp de sécurité
                                    vxCmd =
                                            MathUtil.clamp(
                                                    vxCmd, -TeleopConstants.kMaxTrackingSpeed, TeleopConstants.kMaxTrackingSpeed);
                                    vyCmd =
                                            MathUtil.clamp(
                                                    vyCmd, -TeleopConstants.kMaxTrackingSpeed, TeleopConstants.kMaxTrackingSpeed);

                                    SmartDashboard.putNumber("Angle to Basket", m_robotDrive.getAngleToBasket().getDegrees());

                                    double rotCmd =
                                            MathUtil.applyDeadband(
                                                    m_robotDrive.getCompensationRotation(m_robotDrive.getAngleToBasket().getDegrees()),
                                                    0.01);

                                    Boolean isOnGoodSide = m_robotDrive.isOnGoodSide();

                                    if (!isOnGoodSide) {
                                        m_robotDrive.conduire(
                                                -MathUtil.applyDeadband(
                                                        m_driverController.getRawAxis(1) * DriveConstants.kVitesse,
                                                        OIConstants.kDriveDeadband),
                                                -MathUtil.applyDeadband(
                                                        m_driverController.getRawAxis(0) * DriveConstants.kVitesse,
                                                        OIConstants.kDriveDeadband),
                                                -MathUtil.applyDeadband(
                                                        -m_driverController.getRawAxis(4) * DriveConstants.kVitesseRotation,
                                                        OIConstants.kDriveDeadband),
                                                true, // fieldRelative : true si contrôle relatif au terrain
                                                false);
                                        return;
                                    }

                                    m_robotDrive.conduire(vxCmd, vyCmd, rotCmd, true, false);
                                },
                                m_robotDrive))
                .onFalse(new SetLedsDefault(m_leds));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
