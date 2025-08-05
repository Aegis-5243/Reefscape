// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Intake extends SubsystemBase {
    public abstract double getVelocity();

    public abstract double getPosition();

    public abstract double getOutputCurrent();

    public abstract double getOutputVoltage();

    public abstract void updateSensors();

    public abstract boolean detectingCoral();

    public abstract void stopIntake();

    private double targetSpeed = 0;

    private double targetPosition = 0;

    public Intake() {
        super();

        ShuffleboardTab tab = Shuffleboard.getTab("Intake");
        tab.addDouble("Current Position", this::getPosition)
                .withPosition(3, 0);
        tab.addDouble("Target Position", () -> targetPosition)
                .withPosition(5, 0);
        tab.addDouble("Current Speed", this::getVelocity)
                .withPosition(3, 1);
        tab.addDouble("Target Speed", this::getTargetSpeed)
                .withPosition(4, 1);
        tab.addDouble("Output voltage", this::getOutputVoltage)
                .withPosition(0, 0);
        tab.addDouble("Output current", this::getOutputCurrent)
                .withPosition(0, 1);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        updateSensors();
    }

    public Command setPositionCmd(double position) {
        return run(() -> setPosition(position))
                .until(() -> Math.abs(getPosition() - position) < 0.2);
    }

    public Command setPositionCmd(DoubleSupplier position) {
        return run(() -> setPosition(position.getAsDouble()))
                .until(() -> Math.abs(getPosition() - position.getAsDouble()) < 0.2);
    }

    public void setPosition(double position) {
        targetPosition = position;
    };

    public double getTargetPosition() {
        return targetPosition;
    }

    public Command setVelocityCmd(double speed) {
        return run(() -> setVelocity(speed));
    }

    public void setVelocity(double speed) {
        targetSpeed = speed;
    }

    public double getTargetSpeed() {
        return targetSpeed;
    }

    public Command stopIntakeCommand() {
        return run(this::stopIntake);
    }
}
