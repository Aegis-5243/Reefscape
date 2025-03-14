// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class Turn extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final DriveSubsystem m_subsystem;
    public double yaw;
    public double heading;
    public double startYaw;
    public final double tolerance = .5;

    /**
     * Creates a new Turn command.
     * <p>Turns the robot to the specified heading.
     *
     * @param subsystem The subsystem used by this command.
     * @param degrees The heading you want to turn to in degrees. -180 <= heading <= 180
     */
    public Turn(DriveSubsystem subsystem, double degrees) {
        m_subsystem = subsystem;
        heading = degrees;
        
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        startYaw = m_subsystem.gyro.getYaw();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        yaw = m_subsystem.gyro.getYaw() - startYaw;
        System.out.println(yaw + ", " + heading);
        if (yaw < heading - tolerance || yaw > heading + tolerance) {
            // formatting diff

            double diff = (heading - yaw) / (10000.0);
            diff = diff > 1 ? 1 : diff;
            diff = diff < -1 ? -1 : diff;
            diff = Math.abs(diff) < .3 ? Math.signum(diff) * .3 : diff;
            m_subsystem.mechDrive(0, 0, diff);;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return !(yaw < heading - tolerance || yaw > heading + tolerance);
    }
}