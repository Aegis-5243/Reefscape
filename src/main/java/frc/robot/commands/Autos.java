// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.Utilities;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public final class Autos {
	/** Example static factory for an autonomous command. */
	public static Command exampleAuto(DriveSubsystem subsystem) {
		return Commands.sequence(subsystem.exampleMethodCommand(), new DriveCommand(subsystem));
	}

	public static Command sysIdRoutine(DriveSubsystem subsystem, RoutineType type) {
		Utilities.time.restart();
		Utilities.velocityDict.clear();
		switch (type) {
			case DYNAMIC:
				return subsystem.sysId.dynamic(SysIdRoutine.Direction.kForward);
		
			case QUASISTATIC:
				return subsystem.sysId.quasistatic(SysIdRoutine.Direction.kForward);

			default:
				throw new RuntimeException("Supply a valid routine type");
		}
	}

	public static enum RoutineType {
		DYNAMIC,
		QUASISTATIC
	}

	private Autos() {
		throw new UnsupportedOperationException("This is a utility class!");
	}
}
