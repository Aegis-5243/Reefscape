package frc.robot.controllers;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.utils.UtilFunctions;

public class DriverControls {
    public static Joystick driverLeft;
    public static Joystick driverRight;

    private DoubleSubscriber deadband;

    public DriverControls() {
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
        return driverRight.getRawButton(6);
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
        return driverLeft.getRawButton(6);
    }

    public boolean getAlignLeft() {
        return driverLeft.getRawButton(3);
    }

    public boolean getAlignRight() {
        return driverLeft.getRawButton(4);
    }

    public boolean getIntake() {
        return driverLeft.getRawButton(1);
    }

    public boolean getOuttake() {
        return driverLeft.getRawButton(2);
    }
}
