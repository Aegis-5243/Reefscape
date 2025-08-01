// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  /* Elevator encoder position conversion factor, rotations to inches
   * Gear reduction is 1:20
   * Chain pitch is 0.25 inches
   * Chain gear is 22t
   * Inner stage moves x2 the chain speed
   * = 0.25 * 22 * 1/20 * 2 = 0.55
   */
  public static final double ELEVATOR_POSITION_CONVERSION_FACTOR = 0.55; // inches
  /* Elevator encoder velocity conversion factor, rpm to in/s
   * Rotations to inches is 0.55
   * = 0.55 / 60
   */
  public static final double ELEVATOR_VELOCITY_CONVERSION_FACTOR = ELEVATOR_POSITION_CONVERSION_FACTOR / 60; // inches per second

	public static final double ELEVATOR_MAX_HEIGHT = 50; // inches

	public static final int ELEVATOR_LEFT_MOTOR = 21;
	public static final int ELEVATOR_RIGHT_MOTOR = 22;

	public static final int ELEVATOR_HALL_EFFECT_PORT = 8;

	public static final double ELEVATOR_MAN_TOLERANCE = .5;

	public static final double ELEVATOR_STILL_PERCENT = 0.02734375;

	public static final double ELEVATOR_STALL_CURRENT = 30.0;
}
