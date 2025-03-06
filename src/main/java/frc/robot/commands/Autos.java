// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public final class Autos {
	
	public static ComplexWidget tmp;
	/** Example static factory for an autonomous command. */
	public static Command exampleAuto(DriveSubsystem subsystem) {
		return Commands.sequence(subsystem.exampleMethodCommand(), new DriveCommand(subsystem));
	}

	public static Command driveSysIdRoutine(DriveSubsystem subsystem, RoutineType type, SysIdRoutine.Direction dir) {
		Command command;
		switch (type) {
			case DYNAMIC:
				command = subsystem.sysId.dynamic(dir);
				break;

			case QUASISTATIC:
				command = subsystem.sysId.quasistatic(dir);
				break;

			default:
				throw new RuntimeException("Supply a valid routine type");
		}
		return new SequentialCommandGroup(
				subsystem.runOnce(() -> {
					subsystem.drive.setSafetyEnabled(false);
				}),
				command,
				subsystem.runOnce(() -> {
					subsystem.drive.setSafetyEnabled(true);
				}));
	}

	
	public static Command sysIdRoutine(SysIdRoutine routine, RoutineType type, SysIdRoutine.Direction dir) {
		
		switch (type) {
			case DYNAMIC:
				return routine.dynamic(dir);

			case QUASISTATIC:
				return routine.quasistatic(dir);

			default:
				throw new RuntimeException("Supply a valid routine type");
		}
	}

	public static enum RoutineType {
		DYNAMIC,
		QUASISTATIC
	}

	public static Command testMotor(Subsystem sub, MotorController motor) {
		return sub.startEnd(() -> {
			motor.set(.5);
			System.out.println("running");
		}, () -> {
			motor.set(0);
		});
	}

	public static Command testMotor(Subsystem sub, MotorController motor, double speed) {
		return sub.startEnd(() -> {
			motor.set(speed);
		}, () -> {
			motor.set(0);
		});
	}

	public static Command testPID(SubsystemBase sub, MotorController motor, PIDController motorPID, Encoder encoder) {
		return sub.startRun(() -> {
			tmp = Shuffleboard.getTab("pid test").add("PID", motorPID);//.withWidget(BuiltInWidgets.kPIDController);
		}, () -> {
			// motorPID.setP(tmp.getDouble(Constants.BL_kP));
			motor.setVoltage(motorPID.calculate(encoder.getRate()));
			// System.out.println(tmp.getDouble(0));
		});

	}

	private Autos() {
		throw new UnsupportedOperationException("This is a utility class!");
	}
}
