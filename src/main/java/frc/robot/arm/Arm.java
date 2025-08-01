package frc.robot.arm;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Arm extends SubsystemBase {

    public abstract double getMotorPosition();

    public abstract double getAngle();

    public abstract boolean getLimitSwitch();
    
    public abstract void setEncoderPosition(double degrees);
    
    public abstract double getOutputCurrent();

    public abstract void stopArm();

    public abstract double getVelocity();

    public abstract void setVelocity(double velocity);
    
    private double targetPosition = 0;
    
    public Arm() {
        super();
        
        ShuffleboardTab tab = Shuffleboard.getTab("Arm");
        tab.addDouble("Current Position", this::getAngle);
        
        tab.addDouble("Target Position", this::getTargetPosition);
        tab.addDouble("Velocity", this::getVelocity);
    }
    
    public void setAngle(double degrees) {
        targetPosition = degrees;
    }
    
    public double getTargetPosition() {
        return targetPosition;
    };

    public Command holdArm() {
        return new HoldArm(this);
    }

}