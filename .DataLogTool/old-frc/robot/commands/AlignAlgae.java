// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.Utilities;

/** An example command that uses an example subsystem. */
public class AlignAlgae extends Command {
	@SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    private PIDController yController, rotController;
	private final DriveSubsystem m_subsystem;
    private Timer dontSeeTagTimer, stopTimer;
    private boolean limelightOdo;
    private int pipeline;
    private double tolerance = 1;

    public AlignAlgae(DriveSubsystem subsystem) {
        // xController = new PIDController(Constants.X_ALGAE_ALIGNMENT_P, 0, 0);  // Vertical movement
        yController = new PIDController(Constants.Y_ALGAE_ALIGNMENT_P, 0, 0);  // Horitontal movement
        rotController = new PIDController(Constants.ROT_ALGAE_ALIGNMENT_P, 0, 0);  // Rotation
        this.m_subsystem = subsystem;
        this.pipeline = Constants.APRIL_TAG_PIPELINE;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        limelightOdo = m_subsystem.odoUseLimelight;
        m_subsystem.odoUseLimelight = false;

        this.stopTimer = new Timer();
        this.stopTimer.start();
        this.dontSeeTagTimer = new Timer();
        this.dontSeeTagTimer.start();
        LimelightHelpers.setPipelineIndex(Constants.FRONT_LIMELIGHT, pipeline);
        rotController.setSetpoint(Constants.ROT_SETPOINT_ALGAE_ALIGNMENT);
        rotController.setTolerance(Constants.ROT_TOLERANCE_ALGAE_ALIGNMENT);

        // xController.setSetpoint(Constants.X_SETPOINT_ALGAE_ALIGNMENT);
        // xController.setTolerance(Constants.X_TOLERANCE_ALGAE_ALIGNMENT);

        yController.setSetpoint(Constants.Y_SETPOINT_ALGAE_ALIGNMENT);
        yController.setTolerance(Constants.Y_TOLERANCE_ALGAE_ALIGNMENT);
    }
    
    

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (LimelightHelpers.getTV(Constants.FRONT_LIMELIGHT) &&
            Utilities.isTagOnReef(LimelightHelpers.getFiducialID(Constants.FRONT_LIMELIGHT)))
        {
            this.dontSeeTagTimer.reset();
            double[] postions = LimelightHelpers.getBotPose_TargetSpace(Constants.FRONT_LIMELIGHT);

            // double xSpeed = xController.calculate(postions[2]);
            double xSpeed = Constants.MAX_SPEED_ALGAE_ALIGNMENT;
            double ySpeed = yController.calculate(postions[0]);
            double rotValue = rotController.calculate(postions[4]);

            double maxSpeed = Constants.MAX_SPEED_ALGAE_ALIGNMENT;
            xSpeed = MathUtil.clamp(xSpeed, -maxSpeed, maxSpeed);
            ySpeed = MathUtil.clamp(ySpeed, -maxSpeed, maxSpeed);

            m_subsystem.mechDrive(xSpeed, ySpeed, rotValue);
            if (!rotController.atSetpoint() ||
                !yController.atSetpoint()) {
                stopTimer.reset();
            }
        } else {
            m_subsystem.mechDrive(Constants.MAX_SPEED_ALGAE_ALIGNMENT, 0, 0);
        }
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
        m_subsystem.odoUseLimelight = limelightOdo;
        m_subsystem.mechDrive(0, 0, 0);
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
        // double x = LimelightHelpers.getTX(Constants.FRONT_LIMELIGHT);
		// return !(x < -tolerance || x > tolerance) && time.hasElapsed(3);
        // return this.dontSeeTagTimer.hasElapsed(Constants.DONT_SEE_TAG_WAIT_TIME) ||
        //     stopTimer.hasElapsed(Constants.POSE_VALIDATION_TIME);
        return false;
	}
}
