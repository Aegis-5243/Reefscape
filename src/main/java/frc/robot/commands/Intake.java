// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.RollerSubsystem;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class Intake extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final RollerSubsystem m_subsystem;
    private int stage;
    private double rotations;
    private double tolerance = .005;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public Intake(RollerSubsystem subsystem) {
        m_subsystem = subsystem;
        stage = 0;
        rotations = -.5;

        System.out.println("CONSTRUCT");
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        stage = 0;
        
        System.out.println("INIT");
        m_subsystem.rollerEncoder.setPosition(0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // m_subsystem.roll();
        System.out.println(stage);
        if (stage == 0) {
            m_subsystem.roller.set(-.19);
            if (m_subsystem.laser.getRange() < 55) {
                stage = 1;
                m_subsystem.roller.set(0);
                m_subsystem.rollerEncoder.setPosition(0);
                m_subsystem.setTargetPosition(Units.Rotations.of(rotations));
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_subsystem.rollerEncoder.setPosition(0);
        m_subsystem.setTargetPosition(Units.Rotations.of(0));
        stage = 0;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {

        return m_subsystem.rollerEncoder.getPosition() > rotations - tolerance && m_subsystem.rollerEncoder.getPosition() < rotations + tolerance && stage == 1;
    }
}
