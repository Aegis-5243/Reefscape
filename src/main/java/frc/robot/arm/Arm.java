package frc.robot.arm;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Arm extends SubsystemBase {

    public abstract void setPosition(double distance);

    public abstract void setPower(double pct);

    public abstract double getMotorPosition();

    public abstract double getPosition();

    public abstract boolean getLimitSwitch();

    public abstract double getTargetPosition();

    public abstract void setEncoderPosition(double position);

    public abstract void setVoltage(double volts);

    public Arm() {
        super();

        ShuffleboardTab tab = Shuffleboard.getTab("Arm");
        tab.addDouble("Current Position", this::getPosition);
    }
}