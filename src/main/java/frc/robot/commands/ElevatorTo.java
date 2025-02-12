// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsytem;

/** An example command that uses an example subsystem. */
public class ElevatorTo extends Command {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
	private final ElevatorSubsytem m_subsystem;
	private double target;

	/**
	 * Creates a new ExampleCommand.
	 *
	 * @param time How long to wait (in seconds)
	 */
	public ElevatorTo(ElevatorLocation loc, ElevatorSubsytem subsystem) {
		this.m_subsystem = subsystem;
		this.target = loc.loc.in(Units.Inches) / Constants.ELEVATOR_HEIGHT_PER_MOTOR_ROT.in(Units.Inches);

		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(m_subsystem);
	}

    public static enum ElevatorLocation {
        INTAKE(Units.Meters.of(0)),
		THROUGH(Units.Meters.of(0)),
        LOW_CORAL(Units.Meters.of(0)),
        MID_CORAL(Units.Meters.of(0)),
        HIGH_CORAL(Units.Meters.of(0));

        private final Distance loc;

        private ElevatorLocation(Distance loc) {
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
		
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return m_subsystem.isStill();
	}
}
