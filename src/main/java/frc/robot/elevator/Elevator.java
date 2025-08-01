package frc.robot.elevator;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Elevator extends SubsystemBase {

    public abstract double getMotorPosition();

    public abstract boolean getLimitSwitch();

    public abstract double getOutputCurrent();

    public abstract void setVelocity(double velocity);
    
    public abstract void setEncoderPosition(double position);
    
    public abstract void stopElevator();
    
    private double targetPosition = 0;
    
    public Elevator() {
        super();
        
        ShuffleboardTab tab = Shuffleboard.getTab("Elevator");
        
        tab.addDouble("Current Position", this::getPosition);
        tab.addDouble("Target Position", this::getTargetPosition);
        tab.addDouble("Velocity", this::getVelocity);
        tab.addBoolean("Limit Switch", this::getLimitSwitch);
        tab.add("Set Position Command 0 inches", setPositionCmd(0));
        tab.add("Set Position Command 10 inches", setPositionCmd(10));
        
    }
    
    @Override
    public void periodic() {
        
    }
    
    public void setPosition(double distance) {
        targetPosition = distance;
    }
    
    public double getPosition() {
        return getMotorPosition();
    }

    public double getTargetPosition() {
        return targetPosition;
    };
    
    public Command setPositionCmd(double position) {
        return run(() -> setPosition(position));
    }
    
    public Command holdElevator() {
        return new HoldElevator(this);
    }
    
    protected abstract double getVelocity();
}
