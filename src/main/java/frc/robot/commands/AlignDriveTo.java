// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;

/** An example command that uses an example subsystem. */
public class AlignDriveTo extends Command {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
	private final DriveSubsystem m_subsystem;
	private DriveInfo info;

	/**
	 * Creates a new ExampleCommand.
	 *
	 * @param time How long to wait (in seconds)
	 */
	public AlignDriveTo(DriveLocation loc, DriveSubsystem subsystem) {
		this.m_subsystem = subsystem;
		this.info = loc.loc;

		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(m_subsystem);
	}

    public static enum DriveLocation {
        LEFT_CORAL(new DriveInfo(Constants.FRONT_LIMELIGHT, 1)),
        RIGHT_CORAL(new DriveInfo(Constants.FRONT_LIMELIGHT, 2)),
        SOURCE(new DriveInfo(Constants.BACK_LIMELIGHT, 3));

        private final DriveInfo loc;

        private DriveLocation(DriveInfo loc) {
            this.loc = loc;
        }
    }

    private static class DriveInfo {
        private String limelight;
        private int pipeline;

        private DriveInfo(String limelight, int pipeline) {
            this.limelight = limelight;
            this.pipeline = pipeline;
        }
    }

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
        
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
		return false;
	}
}
