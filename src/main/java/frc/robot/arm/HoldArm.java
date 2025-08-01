package frc.robot.arm;

import edu.wpi.first.wpilibj2.command.Command;


public class HoldArm extends Command {
    private Arm arm;

    private double startPos;

    public HoldArm(Arm arm) {
        this.arm = arm;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        startPos = arm.getAngle();
    }

    @Override
    public void execute() {
        arm.setAngle(startPos);
    }

}
