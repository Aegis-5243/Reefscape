package frc.robot.util;

import frc.robot.Constants;

public class Utilities {
    public static double inchesToRotations(double inches) {
        return (inches / (Math.PI * Constants.WHEEL_DIAMETER));
    }

    public static double rotationsToInches(double rotations) {
        return rotations * (Math.PI * Constants.WHEEL_DIAMETER);
    }

    // TODO: implement
    public static double linearVelocity(double rotations) {  return Double.NaN;  }
}
