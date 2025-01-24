// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.concurrent.CancellationException;

import com.playingwithfusion.CANVenom;
import com.playingwithfusion.CANVenom.BrakeCoastMode;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
	/** Creates a new ExampleSubsystem. */

	public CANVenom fl = new CANVenom(Constants.FL);
	public CANVenom fr = new CANVenom(Constants.FR);
	public CANVenom bl = new CANVenom(Constants.BL);
	public CANVenom br = new CANVenom(Constants.BR);

	public MecanumDrive drive = new MecanumDrive(fl, bl, fr, br);

	public DriveSubsystem() {
		fl.setBrakeCoastMode(BrakeCoastMode.Brake);
		fr.setBrakeCoastMode(BrakeCoastMode.Brake);
		bl.setBrakeCoastMode(BrakeCoastMode.Brake);
		br.setBrakeCoastMode(BrakeCoastMode.Brake);
	}

	/**
	 * Uses input to drive the mechanum chassis
	 * @param xSpeed The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
	 * @param ySpeed The robot's speed along the Y axis [-1.0..1.0]. Left is positive.
	 * @param zSpeed The robot's rotation rate around the Z axis [-1.0..1.0]. Counterclockwise is positive.
	 */
	public void mechDrive(double xSpeed, double ySpeed, double zSpeed) {
		drive.driveCartesian(xSpeed, ySpeed, zSpeed);;
	}

	/**
	 * Uses joysticks to drive the mechanum chassis
	*/
	public void mechDrive() {
		mechDrive(Constants.primaryStick.getX(), Constants.primaryStick.getY(), Constants.secondaryStick.getY());
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
