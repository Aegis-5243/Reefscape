// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

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
	// public static final int FL = 12;
	// public static final int FR = 11;
	// public static final int BL = 14;
	// public static final int BR = 13;
	public static final int FL = 12;
	public static final int FR = 11;
	public static final int BL = 14;
	public static final int BR = 13;

	public static final int[] FL_ENCODER_PORTS = { 7, 6 };
	public static final int[] FR_ENCODER_PORTS = { 1, 0 };
	public static final int[] BL_ENCODER_PORTS = { 5, 4 };
	public static final int[] BR_ENCODER_PORTS = { 3, 2 };

	public static final double FR_kA = 0.32;
	public static final double FR_kS = 0;
	public static final double FR_kV = 3.26;

	public static final double FL_kA = 0.32;
	public static final double FL_kS = 0;
	public static final double FL_kV = 3.26;

	public static final double BR_kA = 0.32;
	public static final double BR_kS = 0;
	public static final double BR_kV = 3.26;

	public static final double BL_kA = 0.32;
	public static final double BL_kS = 0;
	public static final double BL_kV = 3.26;

	public static final double FL_kP = 0.017002;
	public static final double FL_kI = 0.0;
	public static final double FL_kD = 0;

	public static final double FR_kP = 0.010445;
	public static final double FR_kI = 0;
	public static final double FR_kD = 0;

	public static final double BL_kP = 1.6878;
	public static final double BR_kI = 0;
	public static final double BL_kD = 0;

	public static final double BR_kP = 0.017002;
	public static final double BL_KI = 0;
	public static final double BR_kD = 0;

	public static final Distance WHEEL_DIAMETER = Units.Inches.of(6);

	public static final Distance TRACK_WIDTH = Units.Inches.of(22.25);
	public static final Distance TRACK_HEIGHT = Units.Inches.of(20.3599);

	public static final double RIO_CONTROL_LOOP = 0.1;

	// public static final Distance ELEVATOR_HEIGHT_PER_MOTOR_ROT =
	// Units.Inches.of(22.0 / 40);
	public static final Distance ELEVATOR_HEIGHT_PER_MOTOR_ROT = Units.Inches.of(0.5758);
	public static final Distance ELEVATOR_MAX_HEIGHT = Units.Inches.of(50);

	public static final int ELEVATOR_PRIMARY = 21;
	public static final int ELEVATOR_SECONDARY = 22;

	public static final int ELEVATOR_HALL_EFFECT_PORT = 8;

	public static final double ELEVATOR_MAN_TOLERANCE = .5;

	public static final double ELEVATOR_STILL_PERCENT = 0.02734375;

	// TODO: Replace with proper value
	public static final AngularVelocity ELEVATOR_MAX_VELOCITY = Units.RPM.of(1);
	public static final AngularAcceleration ELEVATOR_MAX_ACCELERATION = Units.RPM.per(Units.Second).of(1);

	// public static final Angle

	public static final int ARM = 31;

	public static final double ARM_GEAR_RATIO = 25;

	public static final Angle ARM_MAX_POS = Units.Degrees.of(360);
	public static final Angle ARM_MIN_POS = Units.Degrees.of(17);

	public static final double ARM_kP = 0.1;
	public static final double ARM_kI = 0;
	public static final double ARM_kD = 0;

	public static final int[] ARM_ENCODER = { 10, 12 };

	public static final int ARM_LIMIT_SWITCH = 9;

	// TODO: Replace with proper value
	public static final AngularVelocity ARM_MAX_VELOCITY = Units.RPM.of(1);
	public static final AngularAcceleration ARM_MAX_ACCELERATION = Units.RPM.per(Units.Second).of(1);

	public static final int ROLLER = 32;

	public static final double ROLLER_kP = 3.596;
	public static final double ROLLER_kI = 0;
	public static final double ROLLER_kD = 0;

	public static final Distance ROLLER_DIAMETER = Units.Inches.of(2);

	public static final AngularVelocity ROLLER_OUT_TARGET_VELOCITY = Units.RPM.of(0);
	public static final AngularVelocity ROLLER_IN_TARGET_VELOCITY = Units.RPM.of(0);

	public static final int TIME_OF_FLIGHT = 33;

	public static final double NEO_1_1_kFF = 1.0 / 473;

	public static final double NEO_ENCODER_COUNTS_PER_REV = 42;
	public static final double THROUGH_BORE_COUNTS_PER_REVOLUTION = 8192;
	public static final double THROUGH_BORE_RESOLUTION = 2048;

	public static final String FRONT_LIMELIGHT = "limelight";
	public static final String BACK_LIMELIGHT = "limelight-two";

	public static final int DRIVER_VIEW_PIPELINE = 3;
	public static final int ODOMETRY_PIPIELINE = 0;
	public static final int APRIL_TAG_PIPELINE = 4;
	public static final int LEFT_CORAL_PIPELINE = 2;
	public static final int RIGHT_CORAL_PIPELINE = 1;
	
	// Auton
	public static final double X_ALGAE_ALIGNMENT_P = 3.3;
	public static final double Y_ALGAE_ALIGNMENT_P = 0.08;
	public static final double ROT_ALGAE_ALIGNMENT_P = 0.010;

	public static final double ROT_SETPOINT_ALGAE_ALIGNMENT = 0;  // Rotation
	public static final double ROT_TOLERANCE_ALGAE_ALIGNMENT = 1;
	public static final double X_SETPOINT_ALGAE_ALIGNMENT = -0.3;  // Vertical pose
	public static final double X_TOLERANCE_ALGAE_ALIGNMENT = 0.02;
	public static final double Y_SETPOINT_ALGAE_ALIGNMENT = 0.0;  // Horizontal pose
	public static final double Y_TOLERANCE_ALGAE_ALIGNMENT = 0.02;
	public static final double MAX_SPEED_ALGAE_ALIGNMENT = 0.20;

	public static final double DONT_SEE_TAG_WAIT_TIME = 0.5;
	public static final double POSE_VALIDATION_TIME = 0.3;

	// Controls
	public static Joystick primaryStick = new Joystick(0);
	public static Joystick secondaryStick = new Joystick(1);

	public static Joystick tertiaryStick = new Joystick(3);

	public static XboxController emergencyController = new XboxController(2);

	public static class OperatorConstants {
		public static final int kDriverControllerPort = 0;
	}
}
