// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Joystick;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
	public static final int FL = 13;
	public static final int FR = 12;
	public static final int BL = 11;
	public static final int BR = 14;

	public static final int[] FL_ENCODER_PORTS = {6, 7};
	public static final int[] FR_ENCODER_PORTS = {2, 3};
	public static final int[] BL_ENCODER_PORTS = {4, 5};
	public static final int[] BR_ENCODER_PORTS = {0, 1};

	public static final double FR_kA = 0;
	public static final double FR_kS = 0;
	public static final double FR_kV = 0;

	public static final double FL_kA = 0;
	public static final double FL_kS = 0;
	public static final double FL_kV = 0;

	public static final double BR_kA = 0;
	public static final double BR_kS = 0;
	public static final double BR_kV = 0;
	
	public static final double BL_kA = 0;
	public static final double BL_kS = 0;
	public static final double BL_kV = 0;

	public static final double THROUGH_BORE_COUNTS_PER_REVOLUTION = 8192;

	public static final Distance WHEEL_DIAMETER = Units.Inches.of(8);

	public static final Distance TRACK_WIDTH = Units.Inches.of(22.5);
	public static final Distance TRACK_HEIGHT = Units.Inches.of(20.5);

	public static final double RIO_CONTROL_LOOP = 0.1;

	public static final Distance ELEVATOR_HEIGHT_PER_MOTOR_ROT = Units.Inches.of(22.0/80);

	public static final int ELEVATOR_PRIMARY = 21;
	public static final int ELEVATOR_SECONDARY = 22;
	
	public static Joystick primaryStick = new Joystick(0);
	public static Joystick secondaryStick = new Joystick(1);


	public static class OperatorConstants {
		public static final int kDriverControllerPort = 0;
	}
}
