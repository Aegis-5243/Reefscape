// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.Utilities;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class EncoderTurn extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final DriveSubsystem m_subsystem;
    public double startYaw;
    public double yaw;
    public double turn = 0;
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
    public EncoderTurn(DriveSubsystem subsystem, Angle angle) {
        this(subsystem, angle, 0.5);
    }
    
    public EncoderTurn(DriveSubsystem subsystem, Angle angle, double speed) {
        m_subsystem = subsystem;
        this.counts = (angle.in(Units.Degrees) * Constants.TRACK_WIDTH.in(Units.Inches)) / (360 * Constants.WHEEL_DIAMETER.in(Units.Inches)) * 1.35;
        this.speed = speed;
        
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        zeroFR = m_subsystem.frEncoder.getPosition();
        zeroBR = m_subsystem.brEncoder.getPosition();
        zeroFL = m_subsystem.flEncoder.getPosition();
        zeroBL = m_subsystem.blEncoder.getPosition();

        startYaw = m_subsystem.gyro.getYaw();

        System.out.println("Drive Start");
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // System.out.println("counts target: " + counts + ", current: " + m_subsystem.fl.getPosition());
        m_subsystem.mechDrive(0, 0, Math.signum(counts) * speed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println("Drive END");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Math.abs(m_subsystem.flEncoder.getPosition() - zeroFL) > Math.abs(counts);// && Math.abs(m_subsystem.frEncoder.get() - zeroFR) > Math.abs(counts) && Math.abs(m_subsystem.blEncoder.get() - zeroBL) > Math.abs(counts) && Math.abs(m_subsystem.brEncoder.get() - zeroBR) > Math.abs(counts);
    }
}