// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.Utilities;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

	public static Command sysIdRoutine(DriveSubsystem subsystem, RoutineType type, SysIdRoutine.Direction dir) {
		Utilities.time.restart();
		Utilities.velocityDict.clear();
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

	public static Command testSparkPID(SubsystemBase sub, SparkMax motor) {
		PIDController motorPID = new PIDController(0, 0, 0);
		tmp = Shuffleboard.getTab("pid test").add("PID", motorPID);
		return sub.startRun(() -> {
			//.withWidget(BuiltInWidgets.kPIDController);
		}, () -> {
			// motorPID.setP(tmp.getDouble(Constants.BL_kP));
			motor.configure(new SparkMaxConfig().apply(new ClosedLoopConfig().p(motorPID.getP(), ClosedLoopSlot.kSlot3).d(motorPID.getD(), ClosedLoopSlot.kSlot3).i(motorPID.getI(), ClosedLoopSlot.kSlot3)), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
			motor.getClosedLoopController().setReference(motorPID.getSetpoint(), ControlType.kPosition);
			// System.out.println(tmp.getDouble(0));
		});

	}

	private Autos() {
		throw new UnsupportedOperationException("This is a utility class!");
	}
}
