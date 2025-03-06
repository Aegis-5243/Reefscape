// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.util.Utilities;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class EncoderDrive extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final DriveSubsystem m_subsystem;
    public double yaw;
    public double turn = 0;
    public double zeroFR;
    public double zeroFL;
    public double zeroBR;
    public double zeroBL;
    public double counts;
    public final double tolerance = .75;

    /**
     * Creates a new EncoderDrive command.
     *
     * @param subsystem The subsystem used by this command.
	 * @param distance Distance to drive
     */
    public EncoderDrive(DriveSubsystem subsystem, Distance distance) {
        m_subsystem = subsystem;
        m_subsystem.gyro.reset();
        this.counts = Utilities.distanceToRotations(distance) * Constants.THROUGH_BORE_COUNTS_PER_REVOLUTION;
        
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        zeroFR = m_subsystem.frEncoder.get();
        zeroFL = m_subsystem.flEncoder.get();
        zeroBR = m_subsystem.brEncoder.get();
        zeroBL = m_subsystem.blEncoder.get();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        yaw = m_subsystem.gyro.getYaw();
        if (yaw + tolerance < 0)
            turn += 0.0005;
        if (yaw - tolerance > 0)
            turn -= 0.0005;
        
        m_subsystem.mechDrive(Math.signum(counts * Constants.THROUGH_BORE_COUNTS_PER_REVOLUTION) * -0.5, 0, turn);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Math.abs(m_subsystem.flEncoder.get() - zeroFL) > Math.abs(counts) && Math.abs(m_subsystem.frEncoder.get() - zeroFR) > Math.abs(counts) && Math.abs(m_subsystem.blEncoder.get() - zeroBL) > Math.abs(counts) && Math.abs(m_subsystem.brEncoder.get() - zeroBR) > Math.abs(counts);
    }
}