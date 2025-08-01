// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.RollerSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class OuttakeTimer extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final RollerSubsystem m_subsystem;
    private Timer time = new Timer();
    private double seconds;

    /**
     * Creates a new OuttakeTimer command.
     * <p>OuttakeTimer the coral with the arm
     * 
     * @param subsystem The subsystem used by this command.
     */
    public OuttakeTimer(RollerSubsystem subsystem, double seconds) {
        m_subsystem = subsystem;
        this.seconds = seconds;
        this.time.start();
        
        // System.out.println("CONSTRUCT");
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        
        System.out.println("OUTTAKE START");
        m_subsystem.rollerEncoder.setPosition(0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // m_subsystem.roll();
        // if (stage == 0) {
        //     m_subsystem.roller.set(-.19);
        //     if (m_subsystem.laser.getRange() > 65) {
            //         stage = 1;
            //         m_subsystem.roller.set(0);
            //         m_subsystem.rollerEncoder.setPosition(0);
            //         m_subsystem.setTargetPosition(Units.Rotations.of(rotations));
            //     }
            // }
        m_subsystem.roller.set(-.25);
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // m_subsystem.rollerEncoder.setPosition(0);
        // m_subsystem.setTargetPosition(Units.Rotations.of(0));
        // stage = 0;
        m_subsystem.roller.set(0);
        
        System.out.println("OUTTAKE END");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return time.hasElapsed(seconds);
    }
}
