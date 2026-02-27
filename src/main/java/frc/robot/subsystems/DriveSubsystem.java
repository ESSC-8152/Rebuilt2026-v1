package frc.robot.subsystems;

import java.io.IOException;
import java.util.Optional;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.TeleopConstants;
import frc.robot.LimelightHelpers;

/**
 * Système de conduite swerve refactoré : noms et commentaires en français,
 * structure plus claire, initialisation explicite du pose estimator.
 */
public class DriveSubsystem extends SubsystemBase {

	// ----- Modules swerve -----
	private final MAXSwerveModule avantGauche = new MAXSwerveModule(
			DriveConstants.kFrontLeftDrivingCanId,
			DriveConstants.kFrontLeftTurningCanId,
			DriveConstants.kFrontLeftChassisAngularOffset);

	private final MAXSwerveModule avantDroite = new MAXSwerveModule(
			DriveConstants.kFrontRightDrivingCanId,
			DriveConstants.kFrontRightTurningCanId,
			DriveConstants.kFrontRightChassisAngularOffset);

	private final MAXSwerveModule arriereGauche = new MAXSwerveModule(
			DriveConstants.kRearLeftDrivingCanId,
			DriveConstants.kRearLeftTurningCanId,
			DriveConstants.kBackLeftChassisAngularOffset);

	private final MAXSwerveModule arriereDroite = new MAXSwerveModule(
			DriveConstants.kRearRightDrivingCanId,
			DriveConstants.kRearRightTurningCanId,
			DriveConstants.kBackRightChassisAngularOffset);

	// ----- Gyro -----
	private final GyroIO m_gyro = new GyroIO();

	// ----- Pose estimator (initialisé dans le constructeur) -----
	private SwerveDrivePoseEstimator poseEstimator;

	// ----- Affichage sur le terrain -----
	private final Field2d field2d = new Field2d();

	// One-time alliance-based startup reset. We wait until the Driver Station
	// reports an alliance (not present when the robot boots standalone) and
	// then apply the deterministic starting pose. This avoids applying the
	// wrong starting heading when DS/FMS isn't connected at boot.
	private boolean allianceInitDone = false;

	// (Removed runtime vision inversion flags — we fix the root cause by
	// ensuring the robot orientation sent to the Limelight matches the
	// coordinate convention expected by the camera.)
	// ----- Contrôleurs PID pour conduite vers une pose -----
	private final PIDController xController = new PIDController(1.5, 0.0, 0.0); // m/s per m
	private final PIDController yController = new PIDController(1.5, 0.0, 0.0); // m/s per m
	private final PIDController thetaController = new PIDController(0.015, 0.0, 0.001); // deg/s per deg

	// ----- Tolérances -----
	private final double positionToleranceMeters = 0.05; // 5 cm
	private final double angleToleranceDegrees = 0.5; // 0.01 degrees

	public DriveSubsystem() {
		// Initialisation des encodeurs / odométrie
		resetEncoders();

		// Initialiser le poseEstimator maintenant que le gyro et les modules existent
		poseEstimator = new SwerveDrivePoseEstimator(
				DriveConstants.kDriveKinematics,
				Rotation2d.fromDegrees(getAngle()),
				new SwerveModulePosition[] { avantGauche.getPosition(), avantDroite.getPosition(),
						arriereGauche.getPosition(), arriereDroite.getPosition() },
				Pose2d.kZero);

	// Réinitialiser l'odométrie à l'origine par défaut mais aligner
	// l'orientation avec le gyro instantané afin d'avoir la bonne
	// heading au démarrage.
	resetOdometry(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(getAngle())));

		// Configurer les PID
		thetaController.enableContinuousInput(-180.0, 180.0);
		xController.setTolerance(positionToleranceMeters);
		yController.setTolerance(positionToleranceMeters);
		thetaController.setTolerance(angleToleranceDegrees);

		// Register field on SmartDashboard
		SmartDashboard.putData("Field", field2d);

		RobotConfig config;
		try {
			config = RobotConfig.fromGUISettings();
		} catch (IOException | ParseException e) {
			throw new RuntimeException("Failed to load RobotConfig from GUI settings", e);
		}

		configureAutoBuilder(config);

		PathPlannerLogging.setLogActivePathCallback((poses) -> {
			field2d.getObject("activePath").setPoses(poses);
		});
		}

	@Override
	public void periodic() {
		// Mettre à jour l'estimation de pose avec les positions de modules
		poseEstimator.update(
				Rotation2d.fromDegrees(getAngle()),
				new SwerveModulePosition[] { avantGauche.getPosition(), avantDroite.getPosition(),
						arriereGauche.getPosition(), arriereDroite.getPosition() });

		// Affichage pour debug
		SmartDashboard.putNumber("Angle Gyro", getAngle());
		SmartDashboard.putNumber("Pose Estimator X", getPose().getX());
		SmartDashboard.putNumber("Pose Estimator Y", getPose().getY());
		SmartDashboard.putNumber("Pose Estimator Theta", getPose().getRotation().getDegrees());	
		
		addVisionPosition("limelight-front");
		addVisionPosition("limelight-back");

		// If we haven't applied the alliance-based starting pose yet, wait until
		// the DriverStation reports an alliance (present when DS/FMS connected)
		// and then apply resetToAllianceStartingPose exactly once.
		if (!allianceInitDone) {
			var maybeAlliance = DriverStation.getAlliance();
			if (maybeAlliance.isPresent()) {
				// Apply deterministic start pose (0° blue / 180° red)
				resetToAllianceStartingPose();
				allianceInitDone = true;
				SmartDashboard.putString("AllianceInit", maybeAlliance.get() == Alliance.Red ? "Red applied" : "Blue applied");
			} else {
				SmartDashboard.putString("AllianceInit", "Waiting for DS");
			}
		} 

		// Mettre à jour la vue Field2d
		field2d.setRobotPose(getPose());
	}

	// ----- Commande des modules -----

	/**
	 * Envoie les états désirés à chaque module.
	 */
	public void setModuleStates(SwerveModuleState[] desiredStates) {
		avantGauche.setDesiredState(desiredStates[0]);
		avantDroite.setDesiredState(desiredStates[1]);
		arriereGauche.setDesiredState(desiredStates[2]);
		arriereDroite.setDesiredState(desiredStates[3]);
	}

	public SwerveModuleState[] getModuleStates() {
		return new SwerveModuleState[] { avantGauche.getState(), avantDroite.getState(),
				arriereGauche.getState(), arriereDroite.getState() };
	}

	/**
	 * Conduite manuelle : inputs normalisés [-1,1]. fieldRelative indique si les
	 * vitesses sont en repère terrain.
	 */
	public void conduire(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean squared) {
		if (squared) {
			xSpeed = xSpeed * Math.abs(xSpeed);
			ySpeed = ySpeed * Math.abs(ySpeed);
			rot = rot * Math.abs(rot);
		}

		double xSpeedMeters = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
		double ySpeedMeters = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
		double rotRad = rot * DriveConstants.kMaxAngularSpeed;

		// Use the instantaneous gyro angle for field-relative conversion rather than
		// the pose estimator. The pose estimator can be corrected by vision and
		// therefore lag or differ from the gyro; using it here can produce
		// unexpected drive behavior on the real robot.
		ChassisSpeeds speeds = fieldRelative
			? ChassisSpeeds.fromFieldRelativeSpeeds(-xSpeedMeters, -ySpeedMeters, rotRad,
				Rotation2d.fromDegrees(getAngle()))
			: new ChassisSpeeds(xSpeedMeters, ySpeedMeters, rotRad);

			setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds));
		}

	public void stop() {
		// Mettre des vitesses nulles aux moteurs
		setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0)));
	}

	/**
	 * Positionner les roues en X pour bloquer le robot.
	 */
	public void setX() {
		avantGauche.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
		avantDroite.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
		arriereGauche.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
		arriereDroite.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
	}

	// ----- Pose estimator / odométrie -----

	public Pose2d getPose() {
		return poseEstimator.getEstimatedPosition();
	}

	public void resetPose(Pose2d pose) {
		resetOdometry(pose);
	}

	public void resetOdometry(Pose2d pose) {
		poseEstimator.resetPosition(Rotation2d.fromDegrees(getAngle()),
				new SwerveModulePosition[] { avantGauche.getPosition(), avantDroite.getPosition(),
						arriereGauche.getPosition(), arriereDroite.getPosition() },
				pose);
	}

	// ----- Limelight helpers -----
	public void addVisionPosition(String nomComplet) {
		double yaw = isRedAlliance() ? getAngle() : getAngle() + 180;
		LimelightHelpers.SetRobotOrientation(nomComplet, yaw, 0, 0, 0, 0, 0);

		// Paramètres de confiance pour la mesure vision
		poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.5, 0.5, 9999999));

		// Use the alliance-aware Limelight helper so the pose is reported in the
		// correct field coordinate system (wpiblue or wpired). Previously this
		// always requested the blue frame which made the robot think it was on
		// the wrong alliance when on red.
		LimelightHelpers.PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(nomComplet);

		boolean doRejectUpdate = false;

		if (poseEstimate == null){
			return;
		}
		if (Math.abs(getRate()) > 720) {
			doRejectUpdate = true;
		}
		if (poseEstimate.tagCount == 0) {
			doRejectUpdate = true;
		}
		
		
		SmartDashboard.putBoolean(nomComplet, !doRejectUpdate);
		if (!doRejectUpdate) {
			// Publish raw vision pose for diagnosis
			SmartDashboard.putNumber(nomComplet + " Raw Vision X", poseEstimate.pose.getX());
			SmartDashboard.putNumber(nomComplet + " Raw Vision Y", poseEstimate.pose.getY());
			SmartDashboard.putNumber(nomComplet + " Raw Vision Theta", poseEstimate.pose.getRotation().getDegrees());

			// Use the pose directly from Limelight (no runtime inversion hacks).
			Pose2d usedPose = poseEstimate.pose;

			SmartDashboard.putNumber(nomComplet + " Used Vision X", usedPose.getX());
			SmartDashboard.putNumber(nomComplet + " Used Vision Y", usedPose.getY());
			SmartDashboard.putNumber(nomComplet + " Used Vision Theta", usedPose.getRotation().getDegrees());

			poseEstimator.addVisionMeasurement(usedPose, poseEstimate.timestampSeconds);
		}
	}

	public void setZeroPosition() {
		resetOdometry(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(getAngle())));
	}

	/**
	 * Reset the odometry to a sensible starting pose depending on alliance.
	 * On the red alliance we rotate the starting heading by 180 degrees so the
	 * robot appears in the mirrored orientation on the Field2d.
	 *
	 * This uses the robot's current alliance as reported by DriverStation.
	 */
	public void resetToAllianceStartingPose() {
		// For a deterministic match start, set the starting heading relative
		// to the alliance: 0° when on Blue (facing Blue wall), 180° when on
		// Red (facing Red wall). This mirrors the FRC field convention and
		// ensures Field2d shows the expected orientation at the start of the
		// match regardless of gyro zeroing differences.
		double startHeading = isRedAlliance() ? 180.0 : 0.0;
		Pose2d start = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(startHeading));
		resetOdometry(start);
	}

	// ----- Encodeurs -----

	public void resetEncoders() {
		avantGauche.resetEncoders();
		arriereGauche.resetEncoders();
		avantDroite.resetEncoders();
		arriereDroite.resetEncoders();
	}

	// ----- Gyro -----

	public double getAngle() {
		return m_gyro.getAngle();
	}

	public double getRate() {
		return m_gyro.getRate();
	}

	public void resetGyro() {
		m_gyro.reset();
	}

	// ----- Path planner helpers -----

	public ChassisSpeeds getRobotRelativeSpeeds() {
		return DriveConstants.kDriveKinematics.toChassisSpeeds(
				avantGauche.getState(),
				avantDroite.getState(),
				arriereGauche.getState(),
				arriereDroite.getState());
	}

	public void conduireChassis(ChassisSpeeds chassisSpeeds) {
		// Discrétiser pour cadence de 20 ms (contrôleurs périodiques)
		ChassisSpeeds target = ChassisSpeeds.discretize(chassisSpeeds, 0.02);
		SwerveModuleState[] states = DriveConstants.kDriveKinematics.toSwerveModuleStates(target);
		setModuleStates(states);
	}

	// ----- Alliance -----

	/**
	 * Vérifie si l'alliance est rouge. Appeler souvent car l'alliance peut être
	 * initialisée après le boot.
	 */
	public boolean isRedAlliance() {
		Optional<Alliance> ally = DriverStation.getAlliance();
		if (ally.isPresent()) {
			return ally.get() == Alliance.Red;

		} else {
			return false;
		}
	}

	@Override
    public void simulationPeriodic() {
        // Calcul théorique
        ChassisSpeeds robotSpeeds = DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
        double angularVelocityDeg = Units.radiansToDegrees(robotSpeeds.omegaRadiansPerSecond);
        double angleChange = angularVelocityDeg * 0.02;

		avantGauche.simulationPeriodic(0.02);
		avantDroite.simulationPeriodic(0.02);
		arriereGauche.simulationPeriodic(0.02);
		arriereDroite.simulationPeriodic(0.02);

        // Mise à jour via le Wrapper
        // On récupère l'ancien angle simulé via getAngle(), on ajoute le delta
        // Attention aux signes : getAngle() retourne déjà l'inverse (-yaw) dans ta logique
        double currentSimAngle = m_gyro.getAngle(); 
        
        // Logique simplifiée pour la sim :
        m_gyro.setSimYaw(currentSimAngle + angleChange); // ou -angleChange selon ta convention CCW/CW
    }

	public Command driveToPoseCommand(Pose2d targetPose) {
		PathConstraints constraints = new PathConstraints(
				AutoConstants.kMaxSpeedMetersPerSecond, 
				AutoConstants.kMaxAccelerationMetersPerSecondSquared, 
				Units.degreesToRadians(540), 
				Units.degreesToRadians(720));

		return AutoBuilder.pathfindToPose(
				targetPose,
				constraints,
				0.0 
		);
	}
	

	public Rotation2d getAngleToBasket() {
		double robotX = getPose().getX();
		double robotY = getPose().getY();

		double basketX; 
		double basketY = TeleopConstants.kBasketY; 

		boolean isRed = isRedAlliance();
		basketX = isRed ? TeleopConstants.kBasketXRed : TeleopConstants.kBasketXBlue;

		boolean isOnGoodSide = isOnGoodSide();

		if (isOnGoodSide) {
			double deltaX = basketX - robotX;
			double deltaY = basketY - robotY;

			double angleRad = Math.atan2(deltaY, deltaX);

			return new Rotation2d(angleRad);
		} else {
			return getPose().getRotation();
		}
	}

	public boolean isOnGoodSide() {
		double robotX = getPose().getX();

		double basketX; 

		boolean isRed = isRedAlliance();
		basketX = isRed ? TeleopConstants.kBasketXRed : TeleopConstants.kBasketXBlue;

		boolean isOnGoodSide = isRed ? (robotX < basketX - 0.2) : (robotX > basketX + 0.2);

		return isOnGoodSide; 
	}

	/**
	 * Calcule une pose située à une distance donnée du panier, sur la ligne
	 * panier -> robot, en orientant le robot vers le panier.
	 * @param distanceMeters distance souhaitée entre le robot et le panier
	 * @return Pose2d cible à cette distance, orientée vers le panier
	 */
	public Pose2d getPoseFacingBasketAtDistance(double distanceMeters) {
		double robotX = getPose().getX();
		double robotY = getPose().getY();

		double basketX;
		double basketY = 4.0;
		boolean isRed = isRedAlliance();
		basketX = isRed ? 5.0 : 12.5;

		double dx = robotX - basketX;
		double dy = robotY - basketY;
		double currentDist = Math.hypot(dx, dy);

		// Sécurité pour éviter division par zéro
		if (currentDist < 1e-3) {
			currentDist = 1e-3;
		}

		// Positionner le robot sur le cercle à distance demandée
		double scale = distanceMeters / currentDist;
		double targetX = basketX + dx * scale;
		double targetY = basketY + dy * scale;

		Rotation2d targetHeading = new Rotation2d(Math.atan2(basketY - targetY, basketX - targetX));
		return new Pose2d(targetX, targetY, targetHeading);
	}

	public double getCompensationRotation(Double targetAngleDegrees) {
		double currentAngle = getPose().getRotation().getDegrees();

		double pidOutput = thetaController.calculate(currentAngle, targetAngleDegrees);

		pidOutput += Math.signum(pidOutput) * 0.005; 

		// Clamp critique : on force la valeur entre -0.50 et 0.50 (50% de Vmax)
		// Cela empêche le robot de recevoir une demande de 400% de vitesse.
		double rotationInput = MathUtil.clamp(pidOutput, -0.50, 0.50);

		return rotationInput;
	}

	/**
	 * Accès au contrôleur d'angle pour les commandes externes.
	 */
	public PIDController getThetaController() {
		return thetaController;
	}

	public PIDController getXController() {
		return xController;
	}

	public void configureAutoBuilder(RobotConfig config) {
		AutoBuilder.configure(
            this::getPose,
            this::resetPose,
            this::getRobotRelativeSpeeds,
            (speeds, feedforwards) -> conduireChassis(speeds),
            new PPHolonomicDriveController(
                    new PIDConstants(1.5, 0.0, 0.0),
                    new PIDConstants(6, 0.0, 0.05)
            ),
            config,
            () -> {
                return true;
            },
            this
    	);
	}
}