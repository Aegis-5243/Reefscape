package frc.robot.arm;

import java.util.HashMap;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.ScoringPositions;
import frc.robot.utils.UtilFunctions;

public abstract class Arm extends SubsystemBase {

    public abstract double getMotorPosition();

    public abstract double getAngle();

    public abstract boolean getLimitSwitch();
    
    public abstract void setEncoderPosition(double degrees);
    
    public abstract double getOutputCurrent();

    public abstract double getOutputVoltage();

    public abstract void stopArm();

    public abstract double getVelocity();

    public abstract void setVelocity(double velocity);
    
    private double targetAngle = 0;

    private HashMap<ScoringPositions, DoubleSupplier> positions;
    
    public Arm() {
        super();
        
        ShuffleboardTab tab = Shuffleboard.getTab("Arm");
        tab.addDouble("Current Position", this::getAngle);
        tab.addDouble("Target Position", this::getTargetAngle);
        tab.addDouble("Velocity", this::getVelocity);
        tab.addDouble("Output Current", this::getOutputCurrent);

        positions = new HashMap<>();

        positions.put(ScoringPositions.LoadingPosition, UtilFunctions.getSettingSub("ArmPos/LoadingPosition", 17));
        positions.put(ScoringPositions.L1Coral, UtilFunctions.getSettingSub("ArmPos/L1Coral", 45));
        positions.put(ScoringPositions.L2Coral, UtilFunctions.getSettingSub("ArmPos/L2Coral", 45));
        positions.put(ScoringPositions.L3Coral, UtilFunctions.getSettingSub("ArmPos/L3Coral,", 45));
        positions.put(ScoringPositions.L4Coral, UtilFunctions.getSettingSub("ArmPos/L4Coral,", 60));
        positions.put(ScoringPositions.L2Algae, UtilFunctions.getSettingSub("ArmPos/L2Algae,", 120));
        positions.put(ScoringPositions.L3Algae, UtilFunctions.getSettingSub("ArmPos/L3Algae", 120));
    }
    
    public void setAngle(double degrees) {
        targetAngle = degrees;
    }
    
    public double getTargetAngle() {
        return targetAngle;
    };

    public Command holdArm() {
        return new HoldArm(this);
    }

    public Command setAngleCmd(double angle) {
        return run(() -> setAngle(angle)).until(() -> Math.abs(this.getAngle() - angle) < 2.);
    }

    public Command setAngleCmd(ScoringPositions position) {
        return setAngleCmd(getSetPosition(position));
    }

    public double getSetPosition(ScoringPositions position) {
        return positions.get(position).getAsDouble();
    }

}