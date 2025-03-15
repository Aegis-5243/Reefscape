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
	 * Creates a new ElevatorTo command.
	 * <p>Moves elevator to specified location.
	 * 
	 * @param loc Position to move the elevator to.
	 * @param subsystem The subsystem used by this command.
	 */
	public ElevatorTo(ElevatorLocation loc, ElevatorSubsytem subsystem) {
		this(loc.loc, subsystem);
	}

	public ElevatorTo(Distance dist, ElevatorSubsytem subsystem) {
		this.m_subsystem = subsystem;
		this.target = dist.in(Units.Inches) / Constants.ELEVATOR_HEIGHT_PER_MOTOR_ROT.in(Units.Inches);

		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(m_subsystem);
	}

    public static enum ElevatorLocation {
        INTAKE(Units.Inches.of(0)),
		THROUGH(Units.Inches.of(3.5)),
        LOW_CORAL(Units.Inches.of(13)),
        MID_CORAL(Units.Inches.of(30)),
        HIGH_CORAL(Units.Inches.of(50));

        private final Distance loc;

        private ElevatorLocation(Distance loc) {
            this.loc = loc;
        }
    }

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		// m_subsystem.setTargetPositionPID(target);
		m_subsystem.setTargetPositionMan(target);
		System.out.println("ElevatorTo Start");
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		m_subsystem.runToSetpoint();
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		
		m_subsystem.elevator(0);
		System.out.println("ElevatorTo done");
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return m_subsystem.isDone();
	}
}
