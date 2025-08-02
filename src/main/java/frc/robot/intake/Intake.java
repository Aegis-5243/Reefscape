// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Intake extends SubsystemBase {
    public abstract double getVelocity();

    public abstract double getPosition();

    public abstract double getOutputCurrent();

    public abstract void updateSensors();

    public abstract boolean detectingCoral();

    private double targetSpeed = 0;

    private double targetPosition = 0;

    public Intake() {
        super();

        ShuffleboardTab tab = Shuffleboard.getTab("Intake");
        tab.addDouble("Current Speed", this::getVelocity);
        tab.addDouble("Target Speed", this::getTargetSpeed);
        tab.addDouble("Current Position", this::getPosition);
        tab.addDouble("Target Position", () -> targetPosition);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        updateSensors();
    }

    public Command setPositionCmd(double position) {
        return run(() -> setPosition(position));
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
}
