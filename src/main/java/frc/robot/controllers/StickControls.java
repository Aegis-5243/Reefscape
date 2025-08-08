package frc.robot.controllers;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.UtilFunctions;

public class StickControls extends DriverControls {
    public static Joystick driverLeft;
    public static Joystick driverRight;

    private DoubleSubscriber deadband;

    public StickControls() {
        driverLeft = new Joystick(0);
        driverRight = new Joystick(1);

        deadband = UtilFunctions.getSettingSub("DriveStick/Deadband", 0.05);
    }

    // Forward
    public double getDriveX() {
        var dead = deadband.get();
        return UtilFunctions.deadband(-driverLeft.getY(), dead);
    }

    // Left
    public double getDriveY() {
        var dead = deadband.get();
        return UtilFunctions.deadband(driverLeft.getX(), dead);
    }

    // Counterclockwise
    public double getTurn() {
        var dead = deadband.get();
        return UtilFunctions.deadband(driverRight.getX(), dead);
    }

    public boolean getLoadingPositionCommand() {
        return driverLeft.getRawButton(4);
    }

    public boolean getL1Command() {
        return driverRight.getRawButton(3);
    }

    public boolean getL2Command() {
        return driverRight.getRawButton(4);
    }

    public boolean getL3Command() {
        return driverRight.getRawButton(5);
    }

    public boolean getL4Command() {
        return driverRight.getRawButton(6);
    }


    public boolean getL2AlgaeCommand() {
        return driverLeft.getRawButton(5);
    }

    public boolean getL3AlgaeCommand() {
        return driverLeft.getRawButton(6);
    }

    public boolean getIntake() {
        return driverRight.getRawButton(2);
    }

    public boolean getOuttake() {
        return driverRight.getRawButton(1);
    }

    public Trigger macroIntakeTrigger() {
        return new Trigger(() -> false);
    }

    public Trigger macroTrigger() {
        return new JoystickButton(driverLeft, 1);
    }

    public Trigger resetOdo() {
        return new JoystickButton(driverLeft, 2);
    }

    @Override
    public Trigger autoTestTrigger() {
        return new Trigger(() -> false);
    }

}
