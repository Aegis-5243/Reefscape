// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

/** An example command that uses an example subsystem. */
public class ArmTo extends Command {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
	private final ArmSubsystem m_subsystem;
	private double target;

	/**
	 * Creates a new ExampleCommand.
	 *
	 * @param time How long to wait (in seconds)
	 */
	public ArmTo(ArmLocation loc, ArmSubsystem subsystem) {
		this.m_subsystem = subsystem;
		this.target = loc.loc.minus(Units.Degrees.of(17)).in(Units.Rotations) * Constants.ARM_GEAR_RATIO;

		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(m_subsystem);
	}

    public static enum ArmLocation {
        INTAKE(Units.Degrees.of(17)),
		THROUGH(Units.Degrees.of(45)),
        LOW_CORAL(Units.Degrees.of(55)),
        MID_CORAL(Units.Degrees.of(55)),
        HIGH_CORAL(Units.Degrees.of(69)),
        DURING_ELEVATOR_MOVEMENT(Units.Degrees.of(60));

        private final Angle loc;

        private ArmLocation(Angle loc) {
            this.loc = loc;
        }
    }

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		m_subsystem.setTargetPosition(target);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		System.out.println("ArmTo Done");
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		
		return m_subsystem.armEncoder.getPosition() - target < .2 && m_subsystem.armEncoder.getPosition() - target > -.2;
	}
}
