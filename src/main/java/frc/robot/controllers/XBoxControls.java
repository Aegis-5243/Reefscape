package frc.robot.controllers;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class XBoxControls extends DriverControls {

    XboxController controller;

    enum dPad {
        UP,
        DOWN,
        LEFT,
        RIGHT
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
    }

    @Override
    public double getDriveX() {
        return -controller.getLeftY();
    }

    @Override
    public double getDriveY() {
        return -controller.getLeftX();
    }

    @Override
    public double getTurn() {
        return -controller.getRightX();
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
        return controller.getRightTriggerAxis() > 0.5;
    }

    @Override
    public Trigger driveToPole() {
        return new Trigger(() -> controller.getLeftTriggerAxis() > 0.5);
    }

    @Override
    public Trigger resetOdo() {
        return new Trigger(() -> controller.getBackButton());
    }
    
}
