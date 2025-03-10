// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

/** An example command that uses an example subsystem. */
public class DriveTo extends Command {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
	private final DriveSubsystem m_subsystem;
	private double x;
	private double y;
	private final double tolerance = .5;

	/**
	 * Creates a new DriveTo command.
	 * <p>Drives to specified location using navX displacement.
	 *
	 * @param x How far to travel vetrically. This is relative to robots current position
	 * @param y How far to travel horizontally. This is relative to robots current position
	 */
	public DriveTo(Distance x, Distance y, DriveSubsystem subsystem) {
		this.m_subsystem = subsystem;

		this.x = x.in(Units.Meters);
		this.y = y.in(Units.Meters);

		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(m_subsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
        this.m_subsystem.gyro.resetDisplacement();
		System.out.println("DriveTo Start");
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		float currX = m_subsystem.gyro.getDisplacementX();
		double xSpeed = 0;
        if (currX < x - tolerance || currX > x + tolerance) {
            // formatting xSpeed

            xSpeed = (currX - x) / (x / 4);
            xSpeed = xSpeed > 1 ? 1 : xSpeed;
            xSpeed = xSpeed < -1 ? -1 : xSpeed;
            xSpeed = Math.abs(xSpeed) < .3 ? Math.signum(xSpeed) * .3 : xSpeed;
        }

		float currY = m_subsystem.gyro.getDisplacementY();
		double ySpeed = 0;
        if (currY < y - tolerance || currY > y + tolerance) {
            // formatting xSpeed

            ySpeed = (currY - y) / (y / 4);
            ySpeed = ySpeed > 1 ? 1 : ySpeed;
            ySpeed = ySpeed < -1 ? -1 : ySpeed;
            ySpeed = Math.abs(ySpeed) < .3 ? Math.signum(ySpeed) * .3 : ySpeed;
        }

		// work on heading implementaion

		m_subsystem.mechDrive(xSpeed, ySpeed, 0);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		System.out.println("DriveTo End");
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		float currX = m_subsystem.gyro.getDisplacementX();
		float currY = m_subsystem.gyro.getDisplacementY();
		return !((currY < y - tolerance || currY > y + tolerance) || (currX < x - tolerance || currX > x + tolerance));
	}
}
