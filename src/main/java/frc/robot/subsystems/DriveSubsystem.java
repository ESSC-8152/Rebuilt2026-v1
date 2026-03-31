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
import edu.wpi.first.math.geometry.Pose3d;
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

import org.littletonrobotics.junction.Logger;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.io.GyroIO;
import frc.robot.subsystems.io.GyroIOInputsAutoLogged;
import frc.robot.subsystems.io.SwerveModuleIO;

public class DriveSubsystem extends SubsystemBase {

	// ----- Modules swerve -----
	private final MAXSwerveModule avantGauche;
	private final MAXSwerveModule avantDroite;
	private final MAXSwerveModule arriereGauche;
	private final MAXSwerveModule arriereDroite;

	// ----- Gyro -----
	private final GyroIO m_gyroIO;
	private final GyroIOInputsAutoLogged m_gyroInputs = new GyroIOInputsAutoLogged();

	// ----- Pose estimator -----
	private SwerveDrivePoseEstimator poseEstimator;

	// ----- Affichage sur le terrain -----
	private final Field2d field2d = new Field2d();

	// ----- Contrôleurs PID -----
	private final PIDController thetaController = new PIDController(7, 0.0, 0.05);

	// ----- Mode Boost -----
	private boolean boostMode = false;

	// En attendant que le Driver Station se connecte
	private boolean allianceInitDone = false;

	public DriveSubsystem(GyroIO gyroIO, SwerveModuleIO flIO, SwerveModuleIO frIO, SwerveModuleIO blIO, SwerveModuleIO brIO) {
		this.m_gyroIO = gyroIO;

		// Initialisation des modules avec leurs IO respectives
		this.avantGauche = new MAXSwerveModule("AvantGauche", flIO, DriveConstants.kFrontLeftChassisAngularOffset);
		this.avantDroite = new MAXSwerveModule("AvantDroite", frIO, DriveConstants.kFrontRightChassisAngularOffset);
		this.arriereGauche = new MAXSwerveModule("ArriereGauche", blIO, DriveConstants.kBackLeftChassisAngularOffset);
		this.arriereDroite = new MAXSwerveModule("ArriereDroite", brIO, DriveConstants.kBackRightChassisAngularOffset);

		// Première lecture du gyro pour initialiser l'odométrie
		m_gyroIO.updateInputs(m_gyroInputs);

		resetEncoders();

		thetaController.enableContinuousInput(-180, 180);
		thetaController.setTolerance(1);

		poseEstimator = new SwerveDrivePoseEstimator(
				DriveConstants.kDriveKinematics,
				Rotation2d.fromDegrees(getAngle()),
				new SwerveModulePosition[] { avantGauche.getPosition(), avantDroite.getPosition(),
						arriereGauche.getPosition(), arriereDroite.getPosition() },
				Pose2d.kZero);

		resetOdometry(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(getAngle())));

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

			Logger.recordOutput("Odometry/ActivePath", poses.toArray(new Pose2d[0]));
		});

		PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            Logger.recordOutput("Odometry/TargetPose", pose);
        });
	}

	@Override
	public void periodic() {
		if (!edu.wpi.first.wpilibj.RobotBase.isReal()) {
			ChassisSpeeds vitesses = DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
			m_gyroIO.addSimulatedAngularVelocity(vitesses.omegaRadiansPerSecond);
		}

		m_gyroIO.updateInputs(m_gyroInputs);
		Logger.processInputs("Drive/Gyro", m_gyroInputs);

		avantGauche.periodic();
		avantDroite.periodic();
		arriereGauche.periodic();
		arriereDroite.periodic();

		poseEstimator.update(
				Rotation2d.fromDegrees(getAngle()),
				new SwerveModulePosition[] { avantGauche.getPosition(), avantDroite.getPosition(),
						arriereGauche.getPosition(), arriereDroite.getPosition() });
		
		addVisionPosition("limelight-front");
		addVisionPosition("limelight-back");

		if (!allianceInitDone) {
			var maybeAlliance = DriverStation.getAlliance();
			if (maybeAlliance.isPresent()) {
				resetToAllianceStartingPose();
				allianceInitDone = true;
			} 
		} 

		Pose2d currentPose = poseEstimator.getEstimatedPosition();
		Logger.recordOutput("Odometry/RobotPose2d", currentPose);
		Pose3d robotPose3d = new Pose3d(currentPose); 
		Logger.recordOutput("Odometry/RobotPose3d", robotPose3d);
		field2d.setRobotPose(getPose());

		Logger.recordOutput("SwerveStates/Measured", getModuleStates());
	}

	// ----- Commande des modules -----

	public void setModuleStates(SwerveModuleState[] desiredStates) {
		Logger.recordOutput("SwerveStates/Setpoints", desiredStates);

		avantGauche.setDesiredState(desiredStates[0]);
		avantDroite.setDesiredState(desiredStates[1]);
		arriereGauche.setDesiredState(desiredStates[2]);
		arriereDroite.setDesiredState(desiredStates[3]);
	}

	public SwerveModuleState[] getModuleStates() {
		return new SwerveModuleState[] { avantGauche.getState(), avantDroite.getState(),
				arriereGauche.getState(), arriereDroite.getState() };
	}

	public void conduire(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean squared) {
		if (squared) {
			xSpeed = xSpeed * Math.abs(xSpeed);
			ySpeed = ySpeed * Math.abs(ySpeed);
			rot = rot * Math.abs(rot);
		}

		double xSpeedMeters = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond * (boostMode ? 1.5 : 1);
		double ySpeedMeters = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond * (boostMode ? 1.5 : 1);
		double rotRad = rot * DriveConstants.kMaxAngularSpeed * (boostMode ? 1.5 : 1);

		ChassisSpeeds speeds = fieldRelative
			? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedMeters, ySpeedMeters, rotRad,
				Rotation2d.fromDegrees(getAngle()))
			: new ChassisSpeeds(xSpeedMeters, ySpeedMeters, rotRad);

		setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds));
	}

	public void stop() {
		setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0)));
	}

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

	public void resetToAllianceStartingPose() {
		double startHeading = isRedAlliance() ? 0.0 : 180.0;
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
		// On lit l'angle depuis AdvantageKit, converti de Radians à Degrés
		return Units.radiansToDegrees(m_gyroInputs.yawPositionRad);
	}

	public double getRate() {
		return Units.radiansToDegrees(m_gyroInputs.yawVelocityRadPerSec);
	}

	public void resetGyro() {
		m_gyroIO.reset();
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
				return isRedAlliance() ? false : true;
			},
			this
		);
	}
}