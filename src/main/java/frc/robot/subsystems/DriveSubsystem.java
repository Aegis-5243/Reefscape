// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.Constants;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.Utilities;

public class DriveSubsystem extends SubsystemBase {

	private static DriveSubsystem instance;

	public SparkMax fl;
	public SparkMax fr;
	public SparkMax bl;
	public SparkMax br;

	public RelativeEncoder flEncoder;
	public RelativeEncoder frEncoder;
	public RelativeEncoder blEncoder;
	public RelativeEncoder brEncoder;

	public SimpleMotorFeedforward flFeedForward;
	public SimpleMotorFeedforward frFeedForward;
	public SimpleMotorFeedforward blFeedForward;
	public SimpleMotorFeedforward brFeedForward;

	public SparkClosedLoopController flPID;
	public SparkClosedLoopController frPID;
	public SparkClosedLoopController blPID;
	public SparkClosedLoopController brPID;

	public MecanumDrive drive;

	public SlewRateLimiter limiter;

	public SysIdRoutine sysId;

	public AHRS gyro;

	public MecanumDriveKinematics kinematics;

	public ChassisSpeeds chassisSpeeds;

	public MecanumDrivePoseEstimator poseEstimator;

	public boolean odoUseLimelight;

	public double offsetHeadingDeg;

	public Field2d field;

	/**
	 * Creates a new DriveSubsystem
	 */
	public DriveSubsystem() {
		this.fl = new SparkMax(Constants.FL, MotorType.kBrushless);
		this.fr = new SparkMax(Constants.FR, MotorType.kBrushless);
		this.bl = new SparkMax(Constants.BL, MotorType.kBrushless);
		this.br = new SparkMax(Constants.BR, MotorType.kBrushless);

		this.flEncoder = fl.getAlternateEncoder();
		this.frEncoder = fr.getAlternateEncoder();
		this.blEncoder = bl.getAlternateEncoder();
		this.brEncoder = br.getAlternateEncoder();

		this.flFeedForward = new SimpleMotorFeedforward(Constants.FL_kS, Constants.FL_kV, Constants.FL_kA);
		this.frFeedForward = new SimpleMotorFeedforward(Constants.FR_kS, Constants.FR_kV, Constants.FR_kA);
		this.blFeedForward = new SimpleMotorFeedforward(Constants.BL_kS, Constants.BL_kV, Constants.BL_kA);
		this.brFeedForward = new SimpleMotorFeedforward(Constants.BR_kS, Constants.BR_kV, Constants.BR_kA);

		this.flPID = fl.getClosedLoopController();
		this.frPID = fr.getClosedLoopController();
		this.blPID = bl.getClosedLoopController();
		this.brPID = br.getClosedLoopController();

		this.drive = new MecanumDrive(fl, bl, fr, br);

		this.sysId = new SysIdRoutine(new SysIdRoutine.Config(), new SysIdRoutine.Mechanism(
				voltage -> {
					fl.setVoltage(voltage.magnitude());
					fr.setVoltage(voltage.magnitude());
					bl.setVoltage(voltage.magnitude());
					br.setVoltage(voltage.magnitude());
					System.out.println("setting: " + voltage.magnitude() + "; getteing " + fl.getBusVoltage());
				},
				log -> {
					log.motor("drive-front-right")
							.voltage(Units.Volts.of(fr.getBusVoltage()))
							.linearPosition(Utilities.rotationsToDistance(flEncoder.getPosition()))
							.linearVelocity(Units.MetersPerSecond.of(frEncoder.getVelocity() * 60 * Math.PI
									* Constants.WHEEL_DIAMETER.in(Units.Meters)));

					log.motor("drive-front-left")
							.voltage(Units.Volts.of(fl.getBusVoltage()))
							.linearPosition(Utilities.rotationsToDistance(flEncoder.getPosition()))
							.linearVelocity(Units.MetersPerSecond.of(flEncoder.getVelocity() * 60 * Math.PI
									* Constants.WHEEL_DIAMETER.in(Units.Meters)));

					log.motor("drive-back-left")
							.voltage(Units.Volts.of(bl.getBusVoltage()))
							.linearPosition(Utilities.rotationsToDistance(blEncoder.getPosition()))
							.linearVelocity(Units.MetersPerSecond.of(blEncoder.getVelocity() * 60 * Math.PI
									* Constants.WHEEL_DIAMETER.in(Units.Meters)));

					log.motor("drive-back-right")
							.voltage(Units.Volts.of(br.getBusVoltage()))
							.linearPosition(Utilities.rotationsToDistance(brEncoder.getPosition()))
							.linearVelocity(Units.MetersPerSecond.of(brEncoder.getVelocity() * 60 * Math.PI
									* Constants.WHEEL_DIAMETER.in(Units.Meters)));
				},
				this));

		this.gyro = new AHRS(NavXComType.kUSB1);

		this.gyro.reset();

		this.offsetHeadingDeg = 0.0;

		// Make it so getAngle is zero when facing red aliiance station
		// Needed for field coordinates, which are based on blue alliance.
		if (DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Blue)
			gyro.setAngleAdjustment(180);

		limiter = new SlewRateLimiter(.5);

		/*
		 * TODO:
		 * - Get max drive speed
		 */
		this.kinematics = new MecanumDriveKinematics(new Translation2d(0.259, 0.283), new Translation2d(0.259, -0.283),
				new Translation2d(-0.259, 0.283), new Translation2d(-0.259, -0.283));

		this.poseEstimator = new MecanumDrivePoseEstimator(kinematics, this.gyro.getRotation2d(),
				new MecanumDriveWheelPositions(), Pose2d.kZero);

		RobotConfig config;

		try {
			config = RobotConfig.fromGUISettings();
		} catch (Exception e) {
			// TODO: handle exception
			config = null;
			e.printStackTrace();
		}

		AutoBuilder.configure(
				this::getPose,
				this::setPose,
				this::getChassisSpeeds,
				this::driveRobotSpeed,
				new PPHolonomicDriveController(
						new PIDConstants(1, 0, 0),
						new PIDConstants(1, 0, 0)),
				config,
				() -> {
					Optional<Alliance> alliance = DriverStation.getAlliance();
					if (alliance.isPresent()) {
						return alliance.get() == DriverStation.Alliance.Red;
					}
					return false;
				},
				this);

		odoUseLimelight = true;

		field = new Field2d();

		SmartDashboard.putData("field", field);

		instance = this;
	}

	/**
	 * Uses input to drive the mechanum chassis (robot centric)
	 * 
	 * @param xSpeed The robot's speed along the X axis [-1.0..1.0]. Forward is
	 *               positive.
	 * @param ySpeed The robot's speed along the Y axis [-1.0..1.0]. Left is
	 *               positive.
	 * @param zSpeed The robot's rotation rate around the Z axis [-1.0..1.0].
	 *               Counterclockwise is positive.
	 */
	public void mechDrive(double xSpeed, double ySpeed, double zSpeed) {
		drive.driveCartesian(xSpeed, ySpeed, zSpeed);
	}

	/**
	 * Applies deadzones and exponential scaling to input and uses them to drive the
	 * robot (robot centric)
	 * 
	 * @param xSpeed The robot's speed along the X axis [-1.0..1.0]. Forward is
	 *               positive.
	 * @param ySpeed The robot's speed along the Y axis [-1.0..1.0]. Left is
	 *               positive.
	 * @param zSpeed The robot's rotation rate around the Z axis [-1.0..1.0].
	 *               Counterclockwise is positive.
	 */
	public void DSMechDrive(double xSpeed, double ySpeed, double zSpeed) {
		// Apply deadzone
		double stickDeadzone = 0.15;
		double squaredMag = xSpeed * xSpeed + ySpeed * ySpeed;
		if (squaredMag < stickDeadzone * stickDeadzone)
			xSpeed = ySpeed = 0;
		else {
			// change xspeed and yspeed to start at a magnitude of 0 when leaving deadzone
			if (squaredMag < 1) {
				double angle = Math.atan2(ySpeed, xSpeed);
				// change mag to be from dz - 1 to 0 - 1
				squaredMag = (Math.sqrt(squaredMag) - stickDeadzone) / (1-stickDeadzone);
				xSpeed = squaredMag * Math.cos(angle);
				ySpeed = squaredMag * Math.sin(angle);
			}
		}
		

		if (Math.abs(zSpeed) < stickDeadzone)
			zSpeed = 0;

	

		// Apply exponential rates
		// if (xSpeed != 0 || ySpeed != 0) {
		// squaredMag = Math.sqrt(squaredMag);
		// xSpeed *= squaredMag;
		// ySpeed *= squaredMag;
		// }
		// if (zSpeed != 0) zSpeed = zSpeed * Math.abs(zSpeed);

		// If throttle on primary controller is active then use omni-directional drive

		if (Constants.primaryStick.getRawButton(9)) {
			
			offsetHeadingDeg = gyro.getYaw();
		}

		if (Constants.primaryStick.getThrottle() < -0.5) {
			double heading = gyro.getYaw();
			// System.out.println(heading - offsetHeadingDeg);
			
			double headingDeg = offsetHeadingDeg - heading;
			double headingRad = headingDeg / 180.0 * Math.PI;

			double rotX = xSpeed * Math.cos(headingRad) - ySpeed * Math.sin(headingRad);
        	double rotY = xSpeed * Math.sin(headingRad) + ySpeed * Math.cos(headingRad);

			xSpeed = rotX;
			ySpeed = rotY;

			// drive.driveCartesian(xSpeed, ySpeed, zSpeed, gyro.getRotation2d().minus(Rotation2d.fromDegrees(offsetHeadingDeg)));
		} 

		drive.driveCartesian(xSpeed, ySpeed, zSpeed);
		
	}

	/**
	 * Uses joysticks to drive the mechanum chassis while using a slew rate limiter
	 * (robot centric)
	 */
	public void mechDriveLimiter() {
		mechDrive(limiter.calculate(-Constants.primaryStick.getY()), limiter.calculate(Constants.primaryStick.getX()),
				limiter.calculate(Constants.secondaryStick.getX()));
	}

	/**
	 * Uses joysticks to drive the mechanum chassis while using deadzones (robot
	 * centric)
	 */
	public void mechDrive() {
		DSMechDrive(-Constants.primaryStick.getY(), Constants.primaryStick.getX(), Constants.secondaryStick.getX());
	}

	/**
	 * Uses input to drive the mechanum chassis (field centric)
	 * 
	 * @param xSpeed The robot's speed along the X axis [-1.0..1.0]. Forward is
	 *               positive.
	 * @param ySpeed The robot's speed along the Y axis [-1.0..1.0]. Left is
	 *               positive.
	 * @param zSpeed The robot's rotation rate around the Z axis [-1.0..1.0].
	 *               Counterclockwise is positive.
	 */
	public void fieldMechDrive(double xSpeed, double ySpeed, double zSpeed) {
		drive.driveCartesian(xSpeed, ySpeed, zSpeed, gyro.getRotation2d());
	}

	/**
	 * Uses joysticks to drive the mechanum chassis (field centric)
	 */
	public void fieldMechDrive() {
		fieldMechDrive(-Constants.primaryStick.getY(), Constants.primaryStick.getX(), Constants.secondaryStick.getX());
	}

	/**
	 * Updates current pose estimate using vision, gyro, and encoder data.
	 * <p>
	 * <b>MUST BE CALLED EVERY LOOP!
	 */
	public void updatePoseEstimate() {
		poseEstimator.update(gyro.getRotation2d(),
				new MecanumDriveWheelPositions(Utilities.rotationsToDistance(flEncoder.getPosition()),
						Utilities.rotationsToDistance(frEncoder.getPosition()),
						Utilities.rotationsToDistance(blEncoder.getPosition()),
						Utilities.rotationsToDistance(brEncoder.getPosition())));

		// Modified from
		// https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-swerve-pose-estimation
		if (odoUseLimelight) {

			// LimelightHelpers.setPipelineIndex(Constants.FRONT_LIMELIGHT, Constants.ODOMETRY_PIPIELINE);
			boolean useMegaTag2 = true; // set to false to use MegaTag1
			boolean doRejectUpdate = false;

			// For Front limelight
			if (useMegaTag2 == false) {
				LimelightHelpers.PoseEstimate mt1 = LimelightHelpers
						.getBotPoseEstimate_wpiBlue(Constants.FRONT_LIMELIGHT);

				if (mt1 != null) {
					if (mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
						if (mt1.rawFiducials[0].ambiguity > .7) {
							doRejectUpdate = true;
						}
						if (mt1.rawFiducials[0].distToCamera > 3) {
							doRejectUpdate = true;
						}
					}
					if (mt1.tagCount == 0) {
						doRejectUpdate = true;
					}

					if (!doRejectUpdate) {
						poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));
						poseEstimator.addVisionMeasurement(
								mt1.pose,
								mt1.timestampSeconds);
					}
				}

			} else if (useMegaTag2 == true) {
				LimelightHelpers.SetRobotOrientation(Constants.FRONT_LIMELIGHT,
						poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
				LimelightHelpers.PoseEstimate mt2 = LimelightHelpers
						.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.FRONT_LIMELIGHT);
				if (mt2 != null) {
					if (Math.abs(gyro.getRate()) > 720) // if our angular velocity is greater than 720 degrees per
														// second,
					// ignore vision updates
					{
						doRejectUpdate = true;
					}
					if (mt2.tagCount == 0) {
						doRejectUpdate = true;
					}
					if (!doRejectUpdate) {
						poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
						poseEstimator.addVisionMeasurement(
								mt2.pose,
								mt2.timestampSeconds);
					}
				}
			}

			// For back limelight
			// LimelightHelpers.setPipelineIndex(Constants.BACK_LIMELIGHT, Constants.ODOMETRY_PIPIELINE);

			doRejectUpdate = false;
			if (useMegaTag2 == false) {
				LimelightHelpers.PoseEstimate mt1 = LimelightHelpers
						.getBotPoseEstimate_wpiBlue(Constants.BACK_LIMELIGHT);
				if (mt1 != null) {
					if (mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
						if (mt1.rawFiducials[0].ambiguity > .7) {
							doRejectUpdate = true;
						}
						if (mt1.rawFiducials[0].distToCamera > 3) {
							doRejectUpdate = true;
						}
					}
					if (mt1.tagCount == 0) {
						doRejectUpdate = true;
					}

					if (!doRejectUpdate) {
						poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));
						poseEstimator.addVisionMeasurement(
								mt1.pose,
								mt1.timestampSeconds);
					}
				}

			} else if (useMegaTag2 == true) {
				LimelightHelpers.SetRobotOrientation(Constants.BACK_LIMELIGHT,
						poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
				LimelightHelpers.PoseEstimate mt2 = LimelightHelpers
						.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.BACK_LIMELIGHT);
				if (mt2 != null) {
					if (Math.abs(gyro.getRate()) > 720) // if our angular velocity is greater than 720 degrees per
														// second,
														// ignore vision updates
					{
						doRejectUpdate = true;
					}
					if (mt2.tagCount == 0) {
						doRejectUpdate = true;
					}
					if (!doRejectUpdate) {
						poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
						poseEstimator.addVisionMeasurement(
								mt2.pose,
								mt2.timestampSeconds);
					}
				}
			}
		}

		field.setRobotPose(poseEstimator.getEstimatedPosition());
	}

	/**
	 * Gets the current pose estimate of the robot. Does not update pose estimate.
	 * 
	 * @return Current pose estimate of the robot.
	 */
	public Pose2d getPose() {
		return poseEstimator.getEstimatedPosition();
	}

	/**
	 * Set the current pose of the robot.
	 * 
	 * @param pose Pose of robot to set.
	 */
	public void setPose(Pose2d pose) {
		poseEstimator.resetPose(pose);
	}

	/**
	 * Get speed of robot chassis using wheel velocities.
	 * 
	 * @return The chassis speed of the robot.
	 */
	public ChassisSpeeds getChassisSpeeds() {
		return kinematics.toChassisSpeeds(
				new MecanumDriveWheelSpeeds(
						Units.MetersPerSecond.of(Units.RPM.of(flEncoder.getVelocity()).in(Units.RadiansPerSecond)
								* Constants.WHEEL_DIAMETER.in(Units.Meters) / 2),
						Units.MetersPerSecond.of(Units.RPM.of(frEncoder.getVelocity()).in(Units.RadiansPerSecond)
								* Constants.WHEEL_DIAMETER.in(Units.Meters) / 2),
						Units.MetersPerSecond.of(Units.RPM.of(blEncoder.getVelocity()).in(Units.RadiansPerSecond)
								* Constants.WHEEL_DIAMETER.in(Units.Meters) / 2),
						Units.MetersPerSecond.of(Units.RPM.of(brEncoder.getVelocity()).in(Units.RadiansPerSecond)
								* Constants.WHEEL_DIAMETER.in(Units.Meters) / 2)));
	}

	/**
	 * Drives the robot at a certain chassis speed. Uses FeedForward.
	 * 
	 * @param speeds Target speed of the chassis.
	 */
	public void driveRobotSpeed(ChassisSpeeds speeds) {
		MecanumDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
		
		fl.setVoltage(flFeedForward.calculate(Units.RPM.of(flEncoder.getVelocity()).in(Units.RadiansPerSecond) * Constants.WHEEL_DIAMETER.in(Units.Meters), wheelSpeeds.frontLeftMetersPerSecond));
		fr.setVoltage(frFeedForward.calculate(Units.RPM.of(frEncoder.getVelocity()).in(Units.RadiansPerSecond) * Constants.WHEEL_DIAMETER.in(Units.Meters), wheelSpeeds.frontRightMetersPerSecond));
		bl.setVoltage(blFeedForward.calculate(Units.RPM.of(blEncoder.getVelocity()).in(Units.RadiansPerSecond) * Constants.WHEEL_DIAMETER.in(Units.Meters), wheelSpeeds.rearLeftMetersPerSecond));
		br.setVoltage(brFeedForward.calculate(Units.RPM.of(brEncoder.getVelocity()).in(Units.RadiansPerSecond) * Constants.WHEEL_DIAMETER.in(Units.Meters), wheelSpeeds.rearRightMetersPerSecond));

		// flZPID.setReference(Units.RadiansPerSecond
		// 		.of(wheelSpeeds.frontLeftMetersPerSecond / (Constants.WHEEL_DIAMETER.in(Units.Meters) / 2))
		// 		.in(Units.RPM),
		// 		ControlType.kVelocity);

		// frPID.setReference(Units.RadiansPerSecond
		// 		.of(wheelSpeeds.frontRightMetersPerSecond / (Constants.WHEEL_DIAMETER.in(Units.Meters) / 2))
		// 		.in(Units.RPM),
		// 		ControlType.kVelocity);

		// blPID.setReference(Units.RadiansPerSecond
		// 		.of(wheelSpeeds.rearLeftMetersPerSecond / (Constants.WHEEL_DIAMETER.in(Units.Meters) / 2))
		// 		.in(Units.RPM),
		// 		ControlType.kVelocity);

		// brPID.setReference(Units.RadiansPerSecond
		// 		.of(wheelSpeeds.rearRightMetersPerSecond / (Constants.WHEEL_DIAMETER.in(Units.Meters) / 2))
		// 		.in(Units.RPM),
		// 		ControlType.kVelocity);

	}

	public static DriveSubsystem getInstance() {
		return instance;
	}

	/**
	 * Example command factory method.
	 *
	 * @return a command
	 */
	public Command exampleMethodCommand() {
		// Inline construction of command goes here.
		// Subsystem::RunOnce implicitly requires `this` subsystem.
		return runOnce(
				() -> {
					/* one-time action goes here */
				});
	}

	/**
	 * An example method querying a boolean state of the subsystem (for example, a
	 * digital sensor).
	 *
	 * @return value of some boolean subsystem state, such as a digital sensor.
	 */
	public boolean exampleCondition() {
		// Query some boolean state, such as a digital sensor.
		return false;
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		if (RobotState.isEnabled()) {
			updatePoseEstimate();
			drive.setSafetyEnabled(false);
			odoUseLimelight = false;
		// 	System.out.println("fl: " + flEncoder.getPosition());
		// 	System.out.println("fr: " + frEncoder.getPosition());
		// 	System.out.println("br: " + brEncoder.getPosition());
		// 	System.out.println("bl: " + blEncoder.getPosition());
		}
	}

	@Override
	public void simulationPeriodic() {
		// This method will be called once per scheduler run during simulation
	}
}
