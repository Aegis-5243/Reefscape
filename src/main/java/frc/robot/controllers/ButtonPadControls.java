package frc.robot.controllers;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;


import edu.wpi.first.wpilibj.XboxController;
import frc.robot.vision.Vision.Sides;

public class ButtonPadControls {
    public XboxController controller;

    public ButtonPadControls() {
        this(4);
    }

    public ButtonPadControls(int port) {
        controller = new XboxController(port);
    }

    public Supplier<Sides> getSideSupplier() {
        // return this::getSelectedPole();
        return () -> {return this.getSelectedSide();};
    }

    /**
     * Gets direction of coral alignment as a supplier
     * True is left
     * False is right
     * Assumes left
     * @return supplier
     */
    public BooleanSupplier getDirSupplier() {
        return () -> {
            return !controller.getLeftBumperButton();
        };
    }

    private Sides getSelectedSide() {
        if (controller.getXButton()) {
            return Sides.IJ;
        } else if (controller.getYButton()) {
            return Sides.GH;
        } else if (controller.getRightBumperButton()) {
            return Sides.EF;
        } else if (controller.getAButton()) {
            return Sides.KL;
        } else if (controller.getBButton()) {
            return Sides.AB;
        } else if (controller.getRightTriggerAxis() > 0.75) {
            return Sides.CD;
        }

        return null;
    }
}
