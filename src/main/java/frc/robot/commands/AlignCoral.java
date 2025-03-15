// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.sound.midi.MidiDevice;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.Utilities;

/** An example command that uses an example subsystem. */
public class AlignCoral extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final DriveSubsystem m_driveSubsystem;
    private final CameraSubsystem m_cameraSubsystem;
    
    private PIDController yController, rotController;
    private int pipeline;
    private int oldPipeline;
    private boolean limelight;
    private boolean turning;
    private Timer time;
    private double tolerance = 2.5;
    private double startYaw;

    /**
     * Creates a new AlignCoral command.
     * <p>
     * Aligns robot with a left coral reef branch horizontally and rotationally
     * (optional)
     * 
     * @param driveSubsystem  Subsystem that handles drive operations
     * @param cameraSubsystem Subsystem that handles camera/vision operations
     */
    public AlignCoral(DriveSubsystem driveSubsystem, CameraSubsystem cameraSubsystem) {
        this(driveSubsystem, cameraSubsystem, Constants.LEFT_CORAL_PIPELINE);
    }

    /**
     * Creates a new AlignCoral command.
     * <p>
     * Aligns robot with a coral reef branch horizontally and rotationally
     * (optional)
     * 
     * @param driveSubsystem  Subsystem that handles drive operations
     * @param cameraSubsystem Subsystem that handles camera/vision operations
     * @param pipeline        Limelight pipeline to use with alignment
     */
    public AlignCoral(DriveSubsystem driveSubsystem, CameraSubsystem cameraSubsystem, int pipeline) {
        this(driveSubsystem, cameraSubsystem, pipeline, false);
    }

    /**
     * Creates a new AlignCoral command.
     * <p>
     * Aligns robot with a coral reef branch horizontally and rotationally
     * (optional)
     * 
     * @param driveSubsystem  Subsystem that handles drive operations
     * @param cameraSubsystem Subsystem that handles camera/vision operations
     * @param pipeline        Limelight pipeline to use with alignment
     * @param turn            Is true if you wish to align robot rotationally with
     *                        coral station.
     */
    public AlignCoral(DriveSubsystem driveSubsystem, CameraSubsystem cameraSubsystem, int pipeline, boolean turn) {
        this.m_driveSubsystem = driveSubsystem;
        this.m_cameraSubsystem = cameraSubsystem;
        this.pipeline = pipeline;
        this.turning = turn;
        yController = new PIDController(Constants.Y_ALGAE_ALIGNMENT_P, 0, 0);  // Horitontal movement
        rotController = new PIDController(Constants.ROT_ALGAE_ALIGNMENT_P, 0, 0);  // Rotation
        this.time = new Timer();
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_driveSubsystem, m_cameraSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        oldPipeline = (int) LimelightHelpers.getCurrentPipelineIndex(Constants.FRONT_LIMELIGHT);
        limelight = m_driveSubsystem.odoUseLimelight;
        m_driveSubsystem.odoUseLimelight = false;
        LimelightHelpers.setPipelineIndex(Constants.FRONT_LIMELIGHT, pipeline);
        // startYaw = m_driveSubsystem.gyro.getYaw();
        rotController.setSetpoint(Constants.ROT_SETPOINT_ALGAE_ALIGNMENT);
        rotController.setTolerance(Constants.ROT_TOLERANCE_ALGAE_ALIGNMENT);


        yController.setSetpoint(Constants.Y_SETPOINT_ALGAE_ALIGNMENT);
        yController.setTolerance(Constants.Y_TOLERANCE_ALGAE_ALIGNMENT);

        time.restart();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (LimelightHelpers.getTV(Constants.FRONT_LIMELIGHT) &&
        Utilities.isTagOnReef(LimelightHelpers.getFiducialID(Constants.FRONT_LIMELIGHT))) {
            // formatting diff

            // double strafe = (x) / (150);
            // strafe = strafe > 1 ? 1 : strafe;
            // strafe = strafe < -1 ? -1 : strafe;
            // strafe = Math.abs(strafe) < .2 ? Math.signum(strafe) * .2 : strafe;

            // double turn = 0;
            // if (turning) {
            //     double aprilTagAngle = LimelightHelpers.getBotPose3d_TargetSpace(Constants.FRONT_LIMELIGHT)
            //             .getRotation().getMeasureAngle().in(Units.Degrees);

            //     turn = (aprilTagAngle) / (45.0);
            //     turn = turn > 1 ? 1 : turn;
            //     turn = turn < -1 ? -1 : turn;
            //     turn = Math.abs(turn) < .25 ? Math.signum(turn) * .25 : turn;
            // }

            double[] postions = LimelightHelpers.getBotPose_TargetSpace(Constants.FRONT_LIMELIGHT);
            double x = LimelightHelpers.getTX(Constants.FRONT_LIMELIGHT);
            // double xSpeed = xController.calculate(postions[2]);
            double xSpeed = Constants.MAX_SPEED_ALGAE_ALIGNMENT;
            // double ySpeed = yController.calculate(postions[0]);
            double ySpeed = -yController.calculate(x);
            double rotValue = rotController.calculate(postions[4]);

            double maxSpeed = Constants.MAX_SPEED_ALGAE_ALIGNMENT;
            xSpeed = MathUtil.clamp(xSpeed, -maxSpeed, maxSpeed);
            ySpeed = MathUtil.clamp(ySpeed, -maxSpeed, maxSpeed);

            m_driveSubsystem.mechDrive(xSpeed, ySpeed, rotValue);
            
        } else {
            m_driveSubsystem.mechDrive(Constants.MAX_SPEED_ALGAE_ALIGNMENT, 0, 0);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        LimelightHelpers.setPipelineIndex(Constants.FRONT_LIMELIGHT, oldPipeline);
        m_driveSubsystem.odoUseLimelight = limelight;
        m_driveSubsystem.mechDrive(0, 0, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // TODO: only works in whileTrue, fix if used in auton
        return false;
        // double x = LimelightHelpers.getTX(Constants.FRONT_LIMELIGHT);
        // return !(x < -tolerance || x > tolerance) && time.hasElapsed(3);
    }
}
