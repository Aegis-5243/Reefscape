// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.util.Utilities.ArmLocation;

/** An example command that uses an example subsystem. */
public class ArmToWPI extends Command {
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
	public ArmToWPI(ArmLocation loc, ArmSubsystem subsystem) {
		// this.m_subsystem = subsystem;
		// this.target = loc.loc.minus(Units.Degrees.of(17)).in(Units.Rotations) * Constants.ARM_GEAR_RATIO;

		// // Use addRequirements() here to declare subsystem dependencies.
		// addRequirements(m_subsystem);
		this(loc.loc, subsystem);
	}
	
	public ArmToWPI(Angle loc, ArmSubsystem subsystem) {
		this.m_subsystem = subsystem;
		this.target = (loc.minus(Units.Degrees.of(17)).in(Units.Rotations));
	
		SmartDashboard.putData(m_subsystem.armWPIPIDcontroller);
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(m_subsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		m_subsystem.armWPIPIDcontroller.setSetpoint(target);
		m_subsystem.setpoint = target;
		m_subsystem.armWPIPIDcontroller.setTolerance(.03);
		System.out.println("aRMtO WPI START");
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		System.out.println(m_subsystem.armAltEncoder.getPosition());
		System.out.println(m_subsystem.armWPIPIDcontroller.getP());
		// m_subsystem.armWPIPIDcontroller.setPID(p, i, d);
		SmartDashboard.putNumber("pid-p", m_subsystem.armWPIPIDcontroller.getP());
		SmartDashboard.putNumber("pid-i", m_subsystem.armWPIPIDcontroller.getI());
		SmartDashboard.putNumber("pid-d", m_subsystem.armWPIPIDcontroller.getD());
		m_subsystem.arm.setVoltage(m_subsystem.armWPIPIDcontroller.calculate(m_subsystem.armAltEncoder.getPosition()));
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		System.out.println("ArmTo Done");
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		// return false;
		return m_subsystem.armWPIPIDcontroller.atSetpoint();
	}
}
