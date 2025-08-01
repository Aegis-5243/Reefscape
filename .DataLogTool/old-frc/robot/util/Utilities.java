package frc.robot.util;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.robot.Constants;

public class Utilities {

    /**
     * Returns number of rotations requires to go a certain distance.
     * 
     * @deprecated Use distanceToRotations instead.
     * @param inches Distance to travel in inches.
     * @return The number of rotations needed to travel the specified distance.
     */
    @Deprecated
    public static double inchesToRotations(double inches) {
        return (inches / (Math.PI * Constants.WHEEL_DIAMETER.in(Units.Inches)));
    }

    /**
     * Returns number of rotations requires to go a certain distance.
     * 
     * @param dist Distance to travel.
     * @return The number of rotations needed to travel the specified distance.
     */
    public static double distanceToRotations(Distance dist) {
        return (dist.in(Units.Inches) / (Math.PI * Constants.WHEEL_DIAMETER.in(Units.Inches)));
    }

    /**
     * Returns the linear distance traveled of the drivetrain.
     * 
     * @deprecated Use rotationsToDistance instead.
     * @param rotations Number of wheel motor rotations.
     * @return The linear distance traveled by a wheel motor.
     */
    @Deprecated
    public static double rotationsToInches(double rotations) {
        return rotations * (Math.PI * Constants.WHEEL_DIAMETER.in(Units.Inches));
    }

    /**
     * Returns the linear distance traveled of the drivetrain.
     * 
     * @param rotations Number of wheel motor rotations.
     * @return The linear distance traveled by a wheel motor.
     */
    public static Distance rotationsToDistance(double rotations) {
        return Units.Inches.of(rotations * (Math.PI * Constants.WHEEL_DIAMETER.in(Units.Inches)));
    }

    /**
     * Test a motor by running it at 25% speed.
     * 
     * @deprecated Use testMotor Command inside Autos static class instead.
     * @param motor the motor you wish to test.
     */
    @Deprecated
    public static void testMotor(MotorController motor) {
        motor.set(.25);
    }

    public static boolean isTagOnReef(double tagId) {
        return (17 <= tagId && tagId <= 22) || (6 <= tagId && tagId <= 11);
    }

    public static enum ElevatorLocation {
        INTAKE(Units.Inches.of(0)),
		THROUGH(Units.Inches.of(3.5)),
        LOW_CORAL(Units.Inches.of(13)),
        MID_CORAL(Units.Inches.of(30)),
        HIGH_CORAL(Units.Inches.of(50));

        public final Distance loc;

        private ElevatorLocation(Distance loc) {
            this.loc = loc;
        }
    }

    public static enum ArmLocation {
        INTAKE(Units.Degrees.of(17)),
		THROUGH(Units.Degrees.of(45)),
        LOW_CORAL(Units.Degrees.of(45)),
        MID_CORAL(Units.Degrees.of(45)),
        HIGH_CORAL(Units.Degrees.of(55)),
        DURING_ELEVATOR_MOVEMENT(Units.Degrees.of(65));

        public final Angle loc;

        private ArmLocation(Angle loc) {
            this.loc = loc;
        }
    }

    private Utilities() {
    }
}
