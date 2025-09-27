package frc.robot.elevator;

import java.util.HashMap;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.ScoringPositions;
import frc.robot.util.UtilFunctions;

public abstract class Elevator extends SubsystemBase {

    public abstract double getMotorPosition();

    public abstract boolean getLimitSwitch();

    public abstract double getOutputCurrent();

    public abstract double getOutputVoltage();

    public abstract double getVelocity();

    public abstract void setEncoderPosition(double position);

    public abstract void stopElevator();

    public abstract void setPower(double power);

    private double targetPosition = 0;

    private HashMap<ScoringPositions, DoubleSupplier> positions;

    public boolean isPosition = false;

    public Elevator() {
        super();

        ShuffleboardTab tab = Shuffleboard.getTab("Elevator");

        tab.addDouble("Current Position", this::getPosition);
        tab.addDouble("Target Position", this::getTargetPosition);
        tab.addDouble("Velocity", this::getVelocity);
        tab.addBoolean("Limit Switch", this::getLimitSwitch);
        tab.addDouble("Output current", this::getOutputCurrent);
        tab.addDouble("Output voltage", this::getOutputVoltage);
        tab.add("Set Position Command 0 inches", setPositionCmd(0));
        tab.add("Set Position Command 10 inches", setPositionCmd(10));
        tab.add("Set Position Command 20 inches", setPositionCmd(20));
        tab.add("Set Position Command 30 inches", setPositionCmd(30));
        tab.add("Set Position Command 40 inches", setPositionCmd(40));
        tab.add("Set Position Command 50 inches", setPositionCmd(50));
        tab.add("Set encoder 0", runOnce(() -> this.setEncoderPosition(0)).withName("Reset elev encoder"));

        positions = new HashMap<>();

        positions.put(ScoringPositions.LoadingPosition, UtilFunctions.getSettingSub("ElevatorPos/LoadingPosition", 0));
        positions.put(ScoringPositions.L1Coral, UtilFunctions.getSettingSub("ElevatorPos/L1Coral", 3.5));
        positions.put(ScoringPositions.L2Coral, UtilFunctions.getSettingSub("ElevatorPos/L2Coral", 13));
        positions.put(ScoringPositions.L3Coral, UtilFunctions.getSettingSub("ElevatorPos/L3Coral,", 30));
        positions.put(ScoringPositions.L4Coral, UtilFunctions.getSettingSub("ElevatorPos/L4Coral,", 50));
        positions.put(ScoringPositions.L2Algae, UtilFunctions.getSettingSub("ElevatorPos/L2Algae,", 9));
        positions.put(ScoringPositions.L3Algae, UtilFunctions.getSettingSub("ElevatorPos/L3Algae", 26));
    }

    @Override
    public void periodic() {

    }

    public void setPosition(double distance) {
        isPosition = true;
        targetPosition = distance;
    }

    public double getPosition() {
        return getMotorPosition();
    }

    public double getTargetPosition() {
        return targetPosition;
    };

    public Command setPositionCmd(double position) {
        return run(() -> setPosition(position))
                .until(() -> Math.abs(getPosition() - position) < 1)
                .withName("Elevator set position " + position);
    }

    public Command setPositionCmd(ScoringPositions position) {
        return setPositionCmd(getSetPosition(position));
    }

    public double getSetPosition(ScoringPositions position) {
        return positions.get(position).getAsDouble();
    }

    public Command holdElevator() {
        return new HoldElevator(this);
    }

    public void setVelocity(double velocity) {
        isPosition = false;
    };

    public Command setVelocityCmd(double velocity) {
        return run(() -> setVelocity(velocity));
    }

    public boolean isInPositionMode() {
        return isPosition;
    }

    public Command setPowerCommand(double power) {
        return run(() -> setPower(power));
    }

}
