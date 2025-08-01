// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.util.Utilities.ArmLocation;

/** An example command that uses an example subsystem. */
public class ArmTo extends Command {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
	private final ArmSubsystem m_subsystem;
	private double target;

	/**
	 * Creates a new ArmTo command.
	 * <p>Turns arm to specified location.
	 * 
	 * @param loc Position to turn arm to.
	 * @param subsystem The subsystem used by this command.
	 */
	public ArmTo(ArmLocation loc, ArmSubsystem subsystem) {
		// this.m_subsystem = subsystem;
		// this.target = loc.loc.minus(Units.Degrees.of(17)).in(Units.Rotations) * Constants.ARM_GEAR_RATIO;

		// // Use addRequirements() here to declare subsystem dependencies.
		// addRequirements(m_subsystem);
		this(loc.loc, subsystem);
	}
	
	public ArmTo(Angle loc, ArmSubsystem subsystem) {
		this.m_subsystem = subsystem;
		this.target = loc.minus(Units.Degrees.of(17)).in(Units.Rotations) * Constants.ARM_GEAR_RATIO;
	
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(m_subsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		m_subsystem.setTargetPosition(target);
		System.out.println("aRMtO START");
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		m_subsystem.checklimitSwitch();
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		System.out.println("ArmTo Done");
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		
		return Math.abs(m_subsystem.armEncoder.getPosition() - target) < .025 || 
			m_subsystem.isStalled();
	}
}
