// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;

/** An example command that uses an example subsystem. */
public class DriveTo extends Command {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
	private final DriveSubsystem m_subsystem;
	private double x;
	private double y;
	private double heading;
	private double tolerance = .5;

	/**
	 * Creates a new ExampleCommand.
	 *
	 * @param time How long to wait (in seconds)
	 */
	public DriveTo(double x, double y, DriveSubsystem subsystem) {
		this.m_subsystem = subsystem;

		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(m_subsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
        this.m_subsystem.gyro.resetDisplacement();
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		float yaw = m_subsystem.gyro.getYaw();
        if (yaw < heading - tolerance || yaw > heading + tolerance) {
            // formatting diff

            double diff = (heading - yaw) / (heading * 2);
            diff = diff > 1 ? 1 : diff;
            diff = diff < -1 ? -1 : diff;
            diff = Math.abs(diff) < .3 ? Math.signum(diff) * .3 : diff;
            m_subsystem.mechDrive(0, 0, -diff);;
        }
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
