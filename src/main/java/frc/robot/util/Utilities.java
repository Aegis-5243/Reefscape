package frc.robot.util;

import edu.wpi.first.units.Units;
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

    private Utilities() {
    }
}
