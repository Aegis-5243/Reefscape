// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import java.util.Set;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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

    protected boolean isPosition;

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

        return new DeferredCommand(() -> setPositionCmd(position.getAsDouble()), Set.of(this));
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

    public void setPower(double power) {
        isPosition = false;
    }

    public Command setPowerCmd(double power) {
        return run(() -> setPower(power));
    }

    public Command stopIntakeCommand() {
        return run(this::stopIntake);
    }

    public boolean isInPositionMode() {
        return isPosition;
    }

    public Command homeCoralCommand() {
        return new SequentialCommandGroup(
                setPowerCmd(0.1)
                        .until(() -> detectingCoral()),
                setPowerCmd(-0.03)
                        .until(() -> !detectingCoral()),
                setPositionCmd(() -> getPosition() + 2.7));
    }

    public Command outtakeCommand() {
        return setPowerCmd(0.3)
                .withDeadline(Commands.waitSeconds(1));
    }
}
