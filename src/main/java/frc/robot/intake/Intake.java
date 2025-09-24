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

    public abstract boolean hasCoral();

    public abstract void stopIntake();

    public abstract boolean detectingCoral1();

    public abstract boolean detectingCoral2();

    private double targetSpeed = 0;

    private double targetPosition = 0;

    protected boolean isPosition;

    protected CoralStates currentCoralState = CoralStates.NONE;

    public Intake() {
        super();

        ShuffleboardTab tab = Shuffleboard.getTab("Intake");
        tab.addDouble("Current Position", this::getPosition);
        tab.addDouble("Target Position", () -> targetPosition);
        tab.addDouble("Current Speed", this::getVelocity);
        tab.addDouble("Target Speed", this::getTargetSpeed);
        tab.addDouble("Output voltage", this::getOutputVoltage);
        tab.addDouble("Output current", this::getOutputCurrent);

        tab.addBoolean("TOF 1", this::detectingCoral1);
        tab.addBoolean("TOF 2", this::detectingCoral2);

        tab.add("Set vel 6", setVelocityCmd(6));
        tab.add("Set vel 3", setVelocityCmd(3));
        tab.add("Set vel 0", setVelocityCmd(0));
        tab.add("Set vel -3", setVelocityCmd(-3));
        tab.add("Set power 0.1", setPowerCmd(0.1));
        tab.add("Set power 0.2", setPowerCmd(0.2));
        tab.add("Set power 0.0", setPowerCmd(0));

        tab.addString("Coral State", () -> getCoralState().name());
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
        return run(() -> setPower(power)).withName("Intake power " + power);
    }

    public Command stopIntakeCommand() {
        return run(this::stopIntake).withName("Stop Intake Default Command");
    }

    public boolean isInPositionMode() {
        return isPosition;
    }

    public Command backupCoralCommand() {
        return runEnd(() -> setPower(-0.1),
                () -> setPower(0.0)).onlyWhile(
                        () -> getCoralState() == CoralStates.OUTWARD ||
                                getCoralState() == CoralStates.SAFE);
    }

    public Command homeCoralCommand() {
        return new SequentialCommandGroup(
                setPowerCmd(0.1)
                        .until(() -> hasCoral()),
                setPowerCmd(-0.06)
                        .until(() -> !hasCoral()),
                setPositionCmd(() -> getPosition() + 3.3)).withName("Home coral command");
    }

    public Command outtakeCommand() {
        return setPowerCmd(0.2)
                .withDeadline(Commands.waitSeconds(1)); // TODO: test if enough time
    }

    public Command reverseOuttakeCommand() {
        return setPowerCmd(-0.3)
                .withDeadline(Commands.waitSeconds(1));
    }

    public enum CoralStates {
        NONE, INWARD, SAFE, OUTWARD
    }

    public CoralStates getCoralState() {
        return currentCoralState;
    }
}
