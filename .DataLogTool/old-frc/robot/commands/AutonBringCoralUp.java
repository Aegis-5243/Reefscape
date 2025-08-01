// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RollerSubsystem;

/** An example command that uses an example subsystem. */
public class AutonBringCoralUp extends Command {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
	private final RollerSubsystem m_subsystem;
	Timer time = new Timer();

	/**
	 * Creates a new AutonBringCoralUp command.
	 * @param sub Subsystem that handles rollers.
	 */
	public AutonBringCoralUp(RollerSubsystem sub) {
		this.m_subsystem = sub;

		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(m_subsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		// m_subsystem.drive.setSafetyEnabled(false);
		// this.timer.restart();
		System.out.println("RollerUp START");
		// m_subsystem.rollerEncoder.setPosition(1);
		// m_subsystem.setTargetPosition(Units.Rotations.of(0));
		time.restart();
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		m_subsystem.roller.set(0.10);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		System.out.println("RollerUp END");
		m_subsystem.roller.set(0);

	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return time.hasElapsed(0.25);
	}
}
