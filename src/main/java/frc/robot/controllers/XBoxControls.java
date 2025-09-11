package frc.robot.controllers;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.UtilFunctions;

public class XBoxControls extends DriverControls {

    public XboxController controller;

    private DoubleSubscriber deadband;

    enum dPad {
        UP, DOWN, LEFT, RIGHT
    }

    private boolean getDPad(dPad dir) {
        switch (dir) {
            case UP:
                return controller.getPOV() == 0;
            case DOWN:
                return controller.getPOV() == 180;
            case LEFT:
                return controller.getPOV() == 270;
            case RIGHT:
                return controller.getPOV() == 90;
            default:
                return false;
        }
    }

    public XBoxControls() {
        controller = new XboxController(3);

        deadband = UtilFunctions.getSettingSub("DriveStick/Deadband", 0.05);

    }

    @Override
    public double getDriveX() {
        var dead = deadband.get();
        return UtilFunctions.deadband(-controller.getLeftY(), dead);
    }

    @Override
    public double getDriveY() {
        var dead = deadband.get();
        return UtilFunctions.deadband(controller.getLeftX(), dead);
    }

    @Override
    public double getTurn() {
        var dead = deadband.get();
        return UtilFunctions.deadband(controller.getRawAxis(2), dead);
    }

    @Override
    public boolean getLoadingPositionCommand() {
        return getDPad(dPad.DOWN);
    }

    @Override
    public boolean getL1Command() {
        return controller.getAButton();
    }

    @Override
    public boolean getL2Command() {
        return controller.getXButton();
    }

    @Override
    public boolean getL3Command() {
        return controller.getBButton();
    }

    @Override
    public boolean getL4Command() {
        return controller.getYButton();
    }

    @Override
    public boolean getL2AlgaeCommand() {
        return getDPad(dPad.RIGHT);
    }

    @Override
    public boolean getL3AlgaeCommand() {
        return getDPad(dPad.UP);
    }

    @Override
    public boolean getIntake() {
        return controller.getRightBumperButton();
    }

    @Override
    public boolean getOuttake() {
        // return controller.getRightTriggerAxis() > 0.5;
        return controller.getRawButton(8);
    }

    @Override
    public Trigger macroIntakeTrigger() {
        return new Trigger(() -> controller.getLeftBumperButton());
    }

    @Override
    public Trigger macroTrigger() {
        // return new Trigger(() -> controller.getLeftTriggerAxis() > 0.5);
        return new Trigger(() -> controller.getRawButton(7));
    }

    @Override
    public Trigger resetOdo() {
        return new Trigger(() -> controller.getRawButton(9));
    }

    public boolean getStopTeleop() {
        return controller.getRawButton(10);
    }

    @Override
    public Trigger autoTestTrigger() {
        return new Trigger(() -> getDPad(dPad.LEFT));
    }
}
