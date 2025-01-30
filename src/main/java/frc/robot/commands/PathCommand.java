// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.MecanumPathPlanner;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class PathCommand extends Command {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
	private final DriveSubsystem m_subsystem;
	public double[][] path;
	public MecanumPathPlanner planner;

	/**
	 * Creates a new ExampleCommand.
	 *
	 * @param subsystem The subsystem used by this command.
	 */
	public PathCommand(DriveSubsystem subsystem, double timeToSpend, double[][] path) {
		m_subsystem = subsystem;
		this.path = path;
		this.planner = new MecanumPathPlanner(path);
		
		
		planner.calculate(timeToSpend, Constants.RIO_CONTROL_LOOP, Constants.TRACK_WIDTH, Constants.TRACK_HEIGHT);
		
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(subsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
        m_subsystem.fl.setVoltage(0);
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
