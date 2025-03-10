// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.LimelightHelpers;

/** An example command that uses an example subsystem. */
public class AlignCoralTMP extends Command {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
	private final DriveSubsystem m_subsystem;
    private int pipeline;
    private Timer time;
    private double tolerance = 1;

    /**
     * Creates a new Wait command.
     * <p>Waits for the specified number of seconds.
     * <p>Disables WPILib motor safty during waiting period.
     *
     * @param time How long to wait (in seconds)
     */
    public AlignCoralTMP(DriveSubsystem subsystem) {
        this(subsystem, 2);
    }

    public AlignCoralTMP(DriveSubsystem subsystem, int pipeline) {
        this.m_subsystem = subsystem;
        this.pipeline = pipeline;
        this.time = new Timer();
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        LimelightHelpers.setPipelineIndex(Constants.FRONT_LIMELIGHT, pipeline);
        time.restart();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double x = LimelightHelpers.getTX(Constants.FRONT_LIMELIGHT);
        if (x < -tolerance || x > tolerance) {
            // formatting diff

            double diff = (x) / (120);
            diff = diff > 1 ? 1 : diff;
            diff = diff < -1 ? -1 : diff;
            diff = Math.abs(diff) < .25 ? Math.signum(diff) * .25 : diff;
            m_subsystem.mechDrive(0, diff, 0);;
        }
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
        LimelightHelpers.setPipelineIndex(Constants.FRONT_LIMELIGHT, 0);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
        double x = LimelightHelpers.getTX(Constants.FRONT_LIMELIGHT);
		return !(x < -tolerance || x > tolerance) && time.hasElapsed(3);
	}
}
