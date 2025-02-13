// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** An example command that uses an example subsystem. */
public class SparkMaxPIDTune extends Command {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
	private final SubsystemBase m_subsystem;
	private SparkMax motor;
	private SparkClosedLoopController PIDcontroller;
	private double currP;
	private double currI;
	private double currD;
	private double currTolerance;
	private SimpleWidget PWidget;
	private SimpleWidget IWidget;
	private SimpleWidget DWidget;
	private SimpleWidget ToleranceWidget;
	private SimpleWidget setpointWidget;

	private ShuffleboardTab tab;
	
		/**
		 * Creates a new ExampleCommand.
		 *
		 * @param time How long to wait (in seconds)
		 */
	public SparkMaxPIDTune(SubsystemBase subsystem, SparkMax controller) {
		this.m_subsystem = subsystem;
		this.motor = controller;
		this.PIDcontroller = motor.getClosedLoopController();
		this.currP = motor.configAccessor.closedLoop.getP();
		this.currI = motor.configAccessor.closedLoop.getI();
		this.currD = motor.configAccessor.closedLoop.getD();
		this.currTolerance = motor.configAccessor.closedLoop.maxMotion.getAllowedClosedLoopError();

		this.tab = Shuffleboard.getTab("PID");

		this.PWidget = this.tab.add("P", currP);
		this.IWidget = this.tab.add("I", currI);
		this.DWidget = this.tab.add("D", currD);
		this.ToleranceWidget = this.tab.add("Tolenerance", currTolerance);
		this.setpointWidget = this.tab.add("setpoint", 50.0);
		this.tab.add("GO!", new InstantCommand(() -> {PIDcontroller.setReference(setpointWidget.getEntry().getDouble(currD), ControlType.kMAXMotionPositionControl);}, m_subsystem));
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(m_subsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if (PWidget.getEntry().getDouble(currP) != currP) {
			currP = PWidget.getEntry().getDouble(currP);
			motor.configure(new SparkMaxConfig().apply(new ClosedLoopConfig().p(currP)), ResetMode.kResetSafeParameters,
					PersistMode.kNoPersistParameters);
		}
		if (IWidget.getEntry().getDouble(currI) != currI) {
			currI = IWidget.getEntry().getDouble(currI);
			motor.configure(new SparkMaxConfig().apply(new ClosedLoopConfig().i(currI)), ResetMode.kResetSafeParameters,
					PersistMode.kNoPersistParameters);
		}
		if (DWidget.getEntry().getDouble(currD) != currD) {
			currD = DWidget.getEntry().getDouble(currD);
			motor.configure(new SparkMaxConfig().apply(new ClosedLoopConfig().d(currD)), ResetMode.kResetSafeParameters,
					PersistMode.kNoPersistParameters);
		}
		if (ToleranceWidget.getEntry().getDouble(currTolerance) != currTolerance) {
			currTolerance = PWidget.getEntry().getDouble(currTolerance);
			motor.configure(
					new SparkMaxConfig().apply(
							new ClosedLoopConfig().apply(new MAXMotionConfig().allowedClosedLoopError(currTolerance))),
					ResetMode.kResetSafeParameters,
					PersistMode.kNoPersistParameters);
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
