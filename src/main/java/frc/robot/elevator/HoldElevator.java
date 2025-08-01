package frc.robot.elevator;

import edu.wpi.first.wpilibj2.command.Command;

public class HoldElevator extends Command {

    private Elevator elevator;
    private double startPos;
    private boolean foundSetpoint;

    /**
     * Creates a new HoldElevator Command.
     * <p>To be used as a default command.
     *
     * @param elevator The elevator subsystem used by this command.
     */
    public HoldElevator(Elevator elevator) {
        this.elevator = elevator;
        addRequirements(elevator);
    }

    // Logic from https://github.com/FRC2832/Robot2832-2025Njord/blob/main/src/main/java/frc/robot/elevator/HoldElevator.java
    @Override
    public void initialize() {
        startPos = elevator.getPosition();
        foundSetpoint = false;
    }

    @Override
    public void execute() {
        // at the beginning, we will command the elevator back to where we start
        // once the elevator decelerates, capture the new setpoint to stop the bounce
        if ((foundSetpoint == false) && (Math.abs(elevator.getVelocity()) < 2)) {
        startPos = elevator.getPosition();
        foundSetpoint = true;
        }

        // command the elevator to hold at the position we saved
        elevator.setPosition(startPos);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
