// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.playingwithfusion.CANVenom;
import com.playingwithfusion.CANVenom.BrakeCoastMode;
import com.revrobotics.spark.SparkMax;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.libs.VelocityEncoder;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.Utilities;

public class DriveSubsystem extends SubsystemBase {
	/** Creates a new ExampleSubsystem. */
	private static DriveSubsystem instance;

	public CANVenom fl;
	public CANVenom fr;
	public CANVenom bl;
	public CANVenom br;

	public SparkMax elevator;
	public SparkMax elevatorMinion;

	public VelocityEncoder flEncoder;
	public VelocityEncoder frEncoder;
	public VelocityEncoder blEncoder;
	public VelocityEncoder brEncoder;

	public SimpleMotorFeedforward flFeedForward;
	public SimpleMotorFeedforward frFeedForward;
	public SimpleMotorFeedforward blFeedForward;
	public SimpleMotorFeedforward brFeedForward;

	public PIDController flPID;
	public PIDController frPID;
	public PIDController blPID;
	public PIDController brPID;

	public MecanumDrive drive;

	public SlewRateLimiter limiter;

	public SysIdRoutine sysId;

	public AHRS gyro;

	public DriveSubsystem() {
		this.fl = new CANVenom(Constants.FL);
		this.fr = new CANVenom(Constants.FR);
		this.bl = new CANVenom(Constants.BL);
		this.br = new CANVenom(Constants.BR);

		this.fl.setBrakeCoastMode(BrakeCoastMode.Brake);
		this.fr.setBrakeCoastMode(BrakeCoastMode.Brake);
		this.bl.setBrakeCoastMode(BrakeCoastMode.Brake);
		this.br.setBrakeCoastMode(BrakeCoastMode.Brake);

		this.fl.setInverted(false);
		this.fr.setInverted(true);
		this.bl.setInverted(false);
		this.br.setInverted(true);

		this.flEncoder = new VelocityEncoder(Constants.FL_ENCODER_PORTS[0], Constants.FL_ENCODER_PORTS[1]);
		this.frEncoder = new VelocityEncoder(Constants.FR_ENCODER_PORTS[0], Constants.FR_ENCODER_PORTS[1]);
		this.blEncoder = new VelocityEncoder(Constants.BL_ENCODER_PORTS[0], Constants.BL_ENCODER_PORTS[1]);
		this.brEncoder = new VelocityEncoder(Constants.BR_ENCODER_PORTS[0], Constants.BR_ENCODER_PORTS[1]);

		this.flEncoder.setReverseDirection(false);
		this.frEncoder.setReverseDirection(true);
		this.blEncoder.setReverseDirection(false);
		this.brEncoder.setReverseDirection(true);

		this.flFeedForward = new SimpleMotorFeedforward(Constants.FL_kS, Constants.FL_kV, Constants.FL_kA);
		this.frFeedForward = new SimpleMotorFeedforward(Constants.FR_kS, Constants.FR_kV, Constants.FR_kA);
		this.blFeedForward = new SimpleMotorFeedforward(Constants.BL_kS, Constants.BL_kV, Constants.BL_kA);
		this.brFeedForward = new SimpleMotorFeedforward(Constants.BR_kS, Constants.BR_kV, Constants.BR_kA);

		this.flPID = new PIDController(Constants.FL_kP, 0, Constants.FL_kD, Robot.kDefaultPeriod);
		this.frPID = new PIDController(Constants.FR_kP, 0, Constants.FR_kD, Robot.kDefaultPeriod);
		this.blPID = new PIDController(Constants.BL_kP, 0, Constants.BL_kD, Robot.kDefaultPeriod);
		this.brPID = new PIDController(Constants.BR_kP, 0, Constants.BR_kD, Robot.kDefaultPeriod);

		Utilities.time.start();

		this.drive = new MecanumDrive(fl, bl, fr, br);

		this.sysId = new SysIdRoutine(new SysIdRoutine.Config(), new SysIdRoutine.Mechanism(
				voltage -> {
					fl.setVoltage(voltage.magnitude());
					fr.setVoltage(voltage.magnitude());
					bl.setVoltage(voltage.magnitude());
					br.setVoltage(voltage.magnitude());
					System.out.println("setting: " + voltage.magnitude() + "; getteing " + fl.getOutputVoltage());
				},
				log -> {
					log.motor("drive-front-right")
							.voltage(Units.Volts.of(fr.getBusVoltage()))
							.linearPosition(frEncoder.getDist())
							.linearVelocity(frEncoder.getLinearVelocity());

					log.motor("drive-front-left")
							.voltage(Units.Volts.of(fl.getBusVoltage()))
							.linearPosition(flEncoder.getDist())
							.linearVelocity(flEncoder.getLinearVelocity());

					log.motor("drive-back-left")
							.voltage(Units.Volts.of(bl.getBusVoltage()))
							.linearPosition(blEncoder.getDist())
							.linearVelocity(blEncoder.getLinearVelocity());

					log.motor("drive-back-right")
							.voltage(Units.Volts.of(br.getBusVoltage()))
							.linearPosition(brEncoder.getDist())
							.linearVelocity(brEncoder.getLinearVelocity());
				},
				this));

		this.gyro = new AHRS(NavXComType.kMXP_SPI);

		this.gyro.reset();

		limiter = new SlewRateLimiter(.5);

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
		;
	}

	/**
	 * Uses joysticks to drive the mechanum chassis (robot centric)
	 */
	public void mechDrive() {
		mechDrive(limiter.calculate(-Constants.primaryStick.getY()), limiter.calculate(Constants.primaryStick.getX()), limiter.calculate(Constants.secondaryStick.getX()));
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
		// System.out.println("fl: " + flEncoder.getLinearVelocity().toLongString()); 
		// if (RobotState.isEnabled())
		// 	System.out.println("br: " + brEncoder.getLinearVelocity().toLongString());
		// System.out.println("br: " + brEncoder.getLinearVelocity().toLongString());
		// System.out.println("fr: " + frEncoder.getRate());
		// System.out.println("bl: " + blEncoder.getRate());
		// System.out.println("br: " + brEncoder.getRate());
	}

	@Override
	public void simulationPeriodic() {
		// This method will be called once per scheduler run during simulation
	}
}
