// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.MecanumPathPlanner;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class PathCommand extends Command {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
	private final DriveSubsystem m_subsystem;
	public double[][] path;
	public MecanumPathPlanner planner;
	public double duration;
	private int index;

	/**
	 * Creates a new ExampleCommand.
	 *
	 * @param subsystem The subsystem used by this command.
	 */
	public PathCommand(DriveSubsystem subsystem, double timeToSpend, double[][] path) {
		m_subsystem = subsystem;
		this.path = path;
		this.planner = new MecanumPathPlanner(path);
		this.duration = timeToSpend;

		planner.calculate(timeToSpend, Robot.kDefaultPeriod, Constants.TRACK_WIDTH.in(Units.Meters),
				Constants.TRACK_HEIGHT.in(Units.Meters));

		index = 0;

		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(subsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		m_subsystem.drive.setSafetyEnabled(false);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		m_subsystem.fl.setVoltage(m_subsystem.flFeedForward.calculate(planner.smoothLeftFrontVelocity[index][1]));
		m_subsystem.fr.setVoltage(m_subsystem.frFeedForward.calculate(planner.smoothRightFrontVelocity[index][1]));
		m_subsystem.bl.setVoltage(m_subsystem.blFeedForward.calculate(planner.smoothLeftRearVelocity[index][1]));
		m_subsystem.br.setVoltage(m_subsystem.brFeedForward.calculate(planner.smoothRightRearVelocity[index][1]));
		index++;
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		m_subsystem.fl.setVoltage(0);
		m_subsystem.fr.setVoltage(0);
		m_subsystem.bl.setVoltage(0);
		m_subsystem.br.setVoltage(0);
		m_subsystem.drive.setSafetyEnabled(true);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
