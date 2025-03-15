// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class TimeDrive extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final DriveSubsystem m_subsystem;
    public double startYaw;
    public double yaw;
    public double turn = 0;
    public Timer time = new Timer();
    public double speed;
    public double zeroFR;
    public double zeroFL;
    public double zeroBR;
    public double zeroBL;
    public double counts;
    public final double tolerance = .75;

    /**
     * Creates a new EncoderDrive command.
     * <p>Drives the robot the specified distance.
     *
     * @param subsystem The subsystem used by this command.
	 * @param distance Distance to drive
     */
    
    public TimeDrive(DriveSubsystem subsystem, double time, double speed) {
        m_subsystem = subsystem;
        this.counts = time;
        this.speed = speed;
        
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        time.restart();
        System.out.println("Drive Start");
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        yaw = m_subsystem.gyro.getYaw();
        if (yaw + tolerance < startYaw)
            turn += 0.0005;
        if (yaw - tolerance > startYaw)
            turn -= 0.0005;
        // System.out.println("counts target: " + counts + ", current: " + m_subsystem.fl.getPosition());
        m_subsystem.mechDrive(speed, 0, turn);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println("Drive END");
        m_subsystem.mechDrive(0,0,0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return time.hasElapsed(counts);
        // return Math.abs(m_subsystem.fl.getPosition() - zeroFL) > Math.abs(counts);// && Math.abs(m_subsystem.frEncoder.get() - zeroFR) > Math.abs(counts) && Math.abs(m_subsystem.blEncoder.get() - zeroBL) > Math.abs(counts) && Math.abs(m_subsystem.brEncoder.get() - zeroBR) > Math.abs(counts);
    }
}