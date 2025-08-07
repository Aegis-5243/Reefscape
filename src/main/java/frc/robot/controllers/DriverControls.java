package frc.robot.controllers;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public abstract class DriverControls {

    public abstract double getDriveX();

    public abstract double getDriveY();

    public abstract double getTurn();

    public abstract boolean getLoadingPositionCommand();

    public abstract boolean getL1Command();

    public abstract boolean getL2Command();

    public abstract boolean getL3Command();

    public abstract boolean getL4Command();

    public abstract boolean getL2AlgaeCommand();

    public abstract boolean getL3AlgaeCommand();

    public abstract boolean getIntake();

    public abstract boolean getOuttake();

    public abstract Trigger macroIntakeTrigger();

    public abstract Trigger autoTestTrigger();

    public abstract Trigger macroTrigger();

    public abstract Trigger resetOdo();
}
