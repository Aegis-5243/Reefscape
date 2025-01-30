// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.playingwithfusion.CANVenom;
import com.playingwithfusion.CANVenom.BrakeCoastMode;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.util.Utilities;

public class DriveSubsystem extends SubsystemBase {
	/** Creates a new ExampleSubsystem. */

	public CANVenom fl;
	public CANVenom fr;
	public CANVenom bl;
	public CANVenom br;

	public Encoder flEncoder;
	public Encoder frEncoder;
	public Encoder blEncoder;
	public Encoder brEncoder;

	public SimpleMotorFeedforward flFeedForward;
	public SimpleMotorFeedforward frFeedForward;
	public SimpleMotorFeedforward blFeedForward;
	public SimpleMotorFeedforward brFeedForward;

	public MecanumDrive drive;

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

		this.flEncoder = new Encoder(Constants.FL_ENCODER_PORTS[0], Constants.FL_ENCODER_PORTS[1]);
		this.frEncoder = new Encoder(Constants.FR_ENCODER_PORTS[0], Constants.FR_ENCODER_PORTS[1]);
		this.blEncoder = new Encoder(Constants.BL_ENCODER_PORTS[0], Constants.BL_ENCODER_PORTS[1]);
		this.brEncoder = new Encoder(Constants.BR_ENCODER_PORTS[0], Constants.BR_ENCODER_PORTS[1]);

		this.blFeedForward = new SimpleMotorFeedforward(0, 0, 0);

		Utilities.time.start();
		
		this.drive = new MecanumDrive(fl, bl, fr, br);

		this.sysId = new SysIdRoutine(new SysIdRoutine.Config(), new SysIdRoutine.Mechanism(
			voltage -> {
				fl.setVoltage(voltage.magnitude());
				fr.setVoltage(-voltage.magnitude());
				bl.setVoltage(voltage.magnitude());
				br.setVoltage(-voltage.magnitude());
			},
			log -> {
				log.motor("drive-front-right")
					.voltage(
						Units.Volts.of(
							fr.get() * RobotController.getBatteryVoltage()
						))
					.linearPosition(
						Units.Inches.of(Utilities.rotationsToInches(frEncoder.get() / Constants.THROUGH_BORE_COUNTS_PER_REVOLUTION)))
					.linearVelocity(
						Units.InchesPerSecond.of(Utilities.linearVelocity(frEncoder)));

				log.motor("drive-front-left")
					.voltage(
						Units.Volts.of(
							fl.get() * RobotController.getBatteryVoltage()
						))
					.linearPosition(
						Units.Inches.of(Utilities.rotationsToInches(flEncoder.get() / Constants.THROUGH_BORE_COUNTS_PER_REVOLUTION)))
					.linearVelocity(
						Units.InchesPerSecond.of(Utilities.linearVelocity(flEncoder)));
		
				log.motor("drive-back-left")
					.voltage(
						Units.Volts.of(
							bl.get() * RobotController.getBatteryVoltage()
						))
					.linearPosition(
						Units.Inches.of(Utilities.rotationsToInches(blEncoder.get() / Constants.THROUGH_BORE_COUNTS_PER_REVOLUTION)))
					.linearVelocity(
						Units.InchesPerSecond.of(Utilities.linearVelocity(blEncoder)));
				
				log.motor("drive-back-right")
					.voltage(
						Units.Volts.of(
							br.get() * RobotController.getBatteryVoltage()
						))
					.linearPosition(
						Units.Inches.of(Utilities.rotationsToInches(brEncoder.get() / Constants.THROUGH_BORE_COUNTS_PER_REVOLUTION)))
					.linearVelocity(
						Units.InchesPerSecond.of(Utilities.linearVelocity(brEncoder)));
			},
			this
		));
		
		this.gyro = new AHRS(NavXComType.kMXP_SPI);

		this.gyro.reset();
	}

	/**
	 * Uses input to drive the mechanum chassis (robot centric)
	 * @param xSpeed The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
	 * @param ySpeed The robot's speed along the Y axis [-1.0..1.0]. Left is positive.
	 * @param zSpeed The robot's rotation rate around the Z axis [-1.0..1.0]. Counterclockwise is positive.
	 */
	public void mechDrive(double xSpeed, double ySpeed, double zSpeed) {
		drive.driveCartesian(xSpeed, ySpeed, zSpeed);;
	}

	/**
	 * Uses joysticks to drive the mechanum chassis (robot centric)
	*/
	public void mechDrive() {
		mechDrive(-Constants.primaryStick.getY(), Constants.primaryStick.getX(), Constants.secondaryStick.getX());
	}

	/**
	 * Uses input to drive the mechanum chassis (field centric)
	 * @param xSpeed The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
	 * @param ySpeed The robot's speed along the Y axis [-1.0..1.0]. Left is positive.
	 * @param zSpeed The robot's rotation rate around the Z axis [-1.0..1.0]. Counterclockwise is positive.
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

	public void testMotor(CANVenom motor) {
		motor.set(.25);
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
	}

	@Override
	public void simulationPeriodic() {
		// This method will be called once per scheduler run during simulation
	}
}
