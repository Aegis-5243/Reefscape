package frc.robot.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.intake.Intake.CoralStates;

public class HomeCoral extends Command {
    private Intake intake;

    private CoralStates currentCoralState;
    private CoralStates previousCoralState;
    private boolean hasPosition;
    private double position;
    private boolean tmpName;

    /*
     * Home the coral so it is being seen by both Time of Flight sensors, call with
     * a second boolean arg to intake
     */
    public HomeCoral(Intake intake) {
        this(intake, false);
    }

    /*
     * Home the coral so it is being seen by both Time of Flight sensors, call with
     * a second boolean arg to intake
     */
    public HomeCoral(Intake intake, boolean tmpName) {
        this.intake = intake;
        addRequirements(intake);
        this.tmpName = tmpName;
    }

    @Override
    public void initialize() {
        currentCoralState = intake.getCoralState();
        previousCoralState = currentCoralState;
        hasPosition = false;
    }

    @Override
    public void execute() {
        double speed = 0;
        currentCoralState = intake.getCoralState();
        // if (currentCoralState != previousCoralState) {
        // if (previousCoralState == CoralStates.NONE && currentCoralState ==
        // CoralStates.INWARD) {
        // hasPosition = true;
        // position = -8.0;
        // } else if (previousCoralState == CoralStates.INWARD && currentCoralState ==
        // CoralStates.SAFE) {
        // hasPosition = true;
        // position = 0.0;
        // }
        // }
        if (hasPosition == false) {
            switch (currentCoralState) {
                case SAFE:
                    speed = 0.0;
                    break;
                case INWARD:
                    speed = 3;
                    break;
                case OUTWARD:
                    speed = -3;
                    break;
                case NONE:
                    if (tmpName)
                        speed = 6;
            }
            intake.setVelocity(speed);
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.setVelocity(0);
    }

    @Override
    public boolean isFinished() {
        return currentCoralState == CoralStates.SAFE;
    }
}
