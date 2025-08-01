// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.LimelightHelpers;

/** An example command that uses an example subsystem. */
public class AlignCoralAuto extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final DriveSubsystem m_driveSubsystem;
    private final CameraSubsystem m_cameraSubsystem;
    private int pipeline;
    private boolean turning;
    // private double startYaw;
    // private double turn;
    private Timer time;
    private double tolerance = 0.5;
    private PIDController rotController = new PIDController(Constants.ROT_ALGAE_ALIGNMENT_P, 0, 0);  // Rotation


    /**
     * Creates a new Wait command.
     * <p>
     * Waits for the specified number of seconds.
     * <p>
     * Disables WPILib motor safty during waiting period.
     *
     * @param time How long to wait (in seconds)
     */
    public AlignCoralAuto(DriveSubsystem driveSubsystem, CameraSubsystem cameraSubsystem) {
        this(driveSubsystem, cameraSubsystem, 2);
    }

    public AlignCoralAuto(DriveSubsystem driveSubsystem, CameraSubsystem cameraSubsystem, int pipeline) {
        this(driveSubsystem, cameraSubsystem, pipeline, false);
    }

    public AlignCoralAuto(DriveSubsystem driveSubsystem, CameraSubsystem cameraSubsystem, int pipeline, boolean turn) {
        this.m_driveSubsystem = driveSubsystem;
        this.m_cameraSubsystem = cameraSubsystem;
        this.pipeline = pipeline;
        // this.turning = turn;
        // this.turn = 0;
        // this.startYaw = m_driveSubsystem.gyro.getYaw();
        rotController.setSetpoint(Constants.ROT_SETPOINT_ALGAE_ALIGNMENT);
        rotController.setTolerance(Constants.ROT_TOLERANCE_ALGAE_ALIGNMENT);
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_driveSubsystem, m_cameraSubsystem);
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        LimelightHelpers.setPipelineIndex(Constants.FRONT_LIMELIGHT, pipeline);
        time = new Timer();
        System.out.println("Align Start");
        time.restart();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double x = LimelightHelpers.getTX(Constants.FRONT_LIMELIGHT);
        // if (x < -tolerance || x > tolerance) {
            // formatting diff
            double turn = LimelightHelpers.getBotPose_TargetSpace(Constants.FRONT_LIMELIGHT)[4];
            double rotValue = rotController.calculate(turn);

            double strafe = (x) / (35.0);
            // System.out.println("calib strafe: " + strafe);
            strafe = strafe > 1 ? 1 : strafe;
            strafe = strafe < -1 ? -1 : strafe;
            // strafe = Math.abs(strafe) < .2 ? Math.signum(strafe) * .2 : strafe;

            // double yaw = m_driveSubsystem.gyro.getYaw()
            // System.out.println(yaw + ", " + heading);

            // if (yaw + tolerance < startYaw)
            //     turn += 0.0005;
            // if (yaw - tolerance > startYaw)
            //     turn -= 0.0005;

            m_driveSubsystem.mechDrive(0, strafe, rotValue);

        // }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        LimelightHelpers.setPipelineIndex(Constants.FRONT_LIMELIGHT, 0);
        m_driveSubsystem.mechDrive(0, 0, 0);
        System.out.println("finshed algae align auto");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        double x = LimelightHelpers.getTX(Constants.FRONT_LIMELIGHT);
        return (!(x < -tolerance || x > tolerance) && time.hasElapsed(2)) || time.hasElapsed(3.5);
    }
}
