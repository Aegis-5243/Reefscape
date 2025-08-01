// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ElevatorSubsytem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class ElevatorDown extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final ElevatorSubsytem m_subsystem;
    private double speed;

    /**
     * Creates a new ElevatorCommand.
     * <p>To be used as a default command.
     *
     * @param subsystem The subsystem used by this command.
     */
    public ElevatorDown(ElevatorSubsytem subsystem) {
        this(subsystem, -.4);
    }

    public ElevatorDown(ElevatorSubsytem subsytem, double speed) {
        m_subsystem = subsytem;
        this.speed = speed;

        addRequirements(m_subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_subsystem.elevator(speed);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_subsystem.elevator(0); 
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_subsystem.isStalled();
    }
}
