package frc.robot.controllers;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.vision.Vision.Poles;

public class ButtonPadControls {
    public XboxController controller;

    public ButtonPadControls() {
        this(4);
    }

    public ButtonPadControls(int port) {
        controller = new XboxController(port);
    }

    public Supplier<Poles> getPoleSupplier() {
        // return this::getSelectedPole();
        return () -> {return this.getSelectedPole();};
    }

    private Poles getSelectedPole() {
        if (controller.getXButton()) {
            return Poles.PoleA;
        } else if (controller.getYButton()) {
            return Poles.PoleB;
        } else if (controller.getRightBumperButton()) {
            return Poles.PoleC;
        } else if (controller.getLeftBumperButton()) {
            return Poles.PoleD;
        } else if (controller.getAButton()) {
            return Poles.PoleE;
        } else if (controller.getBButton()) {
            return Poles.PoleF;
        } else if (controller.getRightTriggerAxis() > .75) {
            return Poles.PoleG;
        } else if (controller.getLeftTriggerAxis() > .75) {
            return Poles.PoleH;
        } else if (controller.getLeftStickButton()) {
            return Poles.PoleI;
        } else if (controller.getLeftY() < -.75) {
            return Poles.PoleJ;
        } else if (controller.getRightStickButton()) {
            return Poles.PoleK;
        } else if (controller.getLeftX() > .75) {
            return Poles.PoleL;
        }

        return null;
    }
}
