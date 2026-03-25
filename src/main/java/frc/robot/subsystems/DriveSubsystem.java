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
import edu.wpi.first.math.controller.PIDController;
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
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.LimelightHelpers;

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

	// ----- Contrôleurs PID -----
	private final PIDController thetaController = new PIDController(7, 0.0, 0.05);

	// ----- Mode Boost -----
	private boolean boostMode = false;

	// En attendant que le Driver Station se connecte
	private boolean allianceInitDone = false;

	public DriveSubsystem() {
		// Initialisation des encodeurs / odométrie
		resetEncoders();

		thetaController.enableContinuousInput(-180, 180);
        thetaController.setTolerance(1);

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
		
		addVisionPosition("limelight-front");
		addVisionPosition("limelight-back");

		// appliquer l'orientation de départ basée sur l'alliance
		if (!allianceInitDone) {
			var maybeAlliance = DriverStation.getAlliance();
			if (maybeAlliance.isPresent()) {
				// (0° blue / 180° red)
				resetToAllianceStartingPose();
				allianceInitDone = true;
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
	 * Conduite
	 */
	public void conduire(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean squared) {
		if (squared) {
			xSpeed = xSpeed * Math.abs(xSpeed);
			ySpeed = ySpeed * Math.abs(ySpeed);
			rot = rot * Math.abs(rot);
		}

		double xSpeedMeters = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond * (boostMode ? 1.25 : 1);
		double ySpeedMeters = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond * (boostMode ? 1.25 : 1);
		double rotRad = rot * DriveConstants.kMaxAngularSpeed * (boostMode ? 1.25 : 1);

		ChassisSpeeds speeds = fieldRelative
			? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedMeters, ySpeedMeters, rotRad,
				Rotation2d.fromDegrees(getAngle()))
			: new ChassisSpeeds(xSpeedMeters, ySpeedMeters, rotRad);

		setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds));
	}

	public void stop() {
		setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0)));
	}

	/**
	 * Positionner les roues en X pour arreter le robot.
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

		poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.5, 0.5, 9999999));

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
			poseEstimator.addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds);
		}
	}

	public void setZeroPosition() {
		resetOdometry(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(getAngle())));
	}

	/**
	 * change l'orientation de départ du robot en fonction de l'alliance
	 */
	public void resetToAllianceStartingPose() {
		// (180° red / 0° blue)
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
		double basketY = AutoConstants.kBasketY; 

		boolean isRed = isRedAlliance();
		basketX = isRed ? AutoConstants.kBasketXRed : AutoConstants.kBasketXBlue;

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
		basketX = isRed ? AutoConstants.kBasketXRed : AutoConstants.kBasketXBlue;

		boolean isOnGoodSide = isRed ? (robotX < basketX - 0.2) : (robotX > basketX + 0.2);

		return isOnGoodSide; 
	}

	public double getCompensationRotation(double targetAngleDegrees) {
		double currentAngle = getPose().getRotation().getDegrees();
		double pidOutput = thetaController.calculate(currentAngle, targetAngleDegrees);
		double rotationInput = MathUtil.clamp(pidOutput, -0.7, 0.7);
		return rotationInput;
	}

	public void setBoostMode(boolean boost) {
		this.boostMode = boost;
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