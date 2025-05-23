// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.RollerSubsystem;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class RollerCommand extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final RollerSubsystem m_subsystem;

    /**
     * Creates a new RollerCommand.
     * <p>To be used as a default command.
     *
     * @param subsystem The subsystem used by this command.
     */
    public RollerCommand(RollerSubsystem subsystem) {
        m_subsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // m_subsystem.roll();

        // if (!Constants.primaryStick.getRawButton(2))
            m_subsystem.setTargetPosition(Units.Rotations.of(0));
        // else
            // m_subsystem.roller.set(.05);
            // m_subsystem.rollerEncoder.setPosition(0);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
