// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

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
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  /* Mechanum Drive */

  public static final int FL = 12;
  public static final int FR = 11;
  public static final int BL = 14;
  public static final int BR = 13;

  public static final int[] FL_ENCODER_PORTS = { 7, 6 };
  public static final int[] FR_ENCODER_PORTS = { 1, 0 };
  public static final int[] BL_ENCODER_PORTS = { 5, 4 };
  public static final int[] BR_ENCODER_PORTS = { 3, 2 };

  /*
   * Mechanum conversions
   * 
   * Gear ratio of 12.75 creates a difference in alternate encoders vs motor
   * encoders
   * alternate = base / 12.75
   * 
   * 
   */

  public static final double MECHANUM_ALTERNATE_POSITION_CONVERSION_FACTOR = Units.Rotations.of(1).in(Units.Radians) *
      Constants.WHEEL_DIAMETER.in(Units.Meters) / 2;
  public static final double MECHANUM_ALTERNATE_VELOCITY_CONVERSION_FACTOR = Units.RPM.of(1).in(Units.RadiansPerSecond)
      *
      Constants.WHEEL_DIAMETER.in(Units.Meters) / 2;;

  public static final double MECHANUM_POSITION_CONVERSION_FACTOR = MECHANUM_ALTERNATE_POSITION_CONVERSION_FACTOR
      / 12.75;
  public static final double MECHANUM_VELOCITY_CONVERSION_FACTOR = MECHANUM_ALTERNATE_VELOCITY_CONVERSION_FACTOR
  / 12.75;;

  public static final double FR_kA = 0.32;
  public static final double FR_kS = 0.13;
  public static double FR_kV = (2.9);

  public static final double FL_kA = 0.32;
  public static final double FL_kS = 0.13;
  public static double FL_kV = (2.9);

  public static final double BR_kA = 0.32;
  public static final double BR_kS = 0.13;
  public static double BR_kV = (2.9);

  public static final double BL_kA = 0.32;
  public static final double BL_kS = 0.13;
  public static double BL_kV = (2.9);

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

  public static final double MECANUM_STRAFE_SPEED_ACCOUNT = 0.82; // The rate at which the robot's strafing velocity
                                                                  // differs from forward velocity with the same input
  public static final double MECANUM_ROTATION_SPEED_ACCOUNT = 1.08; // Multiplier for rotation speed to account for the
                                                                    // fact that mecanum wheels rotate slower

  public static final double RIO_CONTROL_LOOP = 0.1;

  /* Elevator */

  /*
   * Elevator conversion factors
   * Gear reduction is 1:20
   * Chain pitch is 0.25 inches
   * Chain gear is 22t
   * Inner stage moves x2 the chain speed
   * = 0.25 * 22 * 1/20 * 2 = 0.55
   */
  public static final double ELEVATOR_POSITION_CONVERSION_FACTOR = 0.55; // inches
  // Convert rpm to inches per second
  public static final double ELEVATOR_VELOCITY_CONVERSION_FACTOR = ELEVATOR_POSITION_CONVERSION_FACTOR / 60;

  public static final double ELEVATOR_MAX_VELOCITY = 12; // inches per second

  public static final double ELEVATOR_kP = 0.1;
  public static final double ELEVATOR_kI = 0;
  public static final double ELEVATOR_kD = 0;

  public static final double ELEVATOR_MAX_HEIGHT = 50; // inches

  public static final int ELEVATOR_LEFT_MOTOR_PORT = 21;
  public static final int ELEVATOR_RIGHT_MOTOR_PORT = 22;

  public static final int ELEVATOR_HALL_EFFECT_PORT = 8;

  public static final double ELEVATOR_MAN_TOLERANCE = .5;

  public static final double ELEVATOR_STILL_PERCENT = 0.02734375;

  public static final double ELEVATOR_STALL_CURRENT = 30.0;

  /* Arm */
  public static final int ARM_MOTOR_PORT = 31;

  /*
   * Arm conversion factors
   * Gear ratio: 1:25
   * Rotations to degrees: 360
   * = 360 / 25 = 14.4
   */
  public static final double ARM_POSITION_CONVERSION_FACTOR = 14.4;
  // Convert rpm to degrees per second
  public static final double ARM_VELOCITY_CONVERSION_FACTOR = ARM_POSITION_CONVERSION_FACTOR / 60;

  public static final double ARM_MAX_POS = 360; // degrees
  public static final double ARM_MIN_POS = 17; // degrees

  public static final double ARM_MAX_VELOCITY = 12; // degrees per second

  public static final double ARM_kP = 0.25;
  public static final double ARM_kI = 0;
  public static final double ARM_kD = 0;

  public static final int ARM_LIMIT_SWITCH_PORT = 9;

  /* Roller */
  public static final int ROLLER_PORT = 32;

  public static final int ROLLER_TIME_OF_FLIGHT_PORT = 33;

  /*
   * Roller conversion factors
   * Belt ratio: 12 / 48
   * Roller diameter: 2 inches
   * = (12 / 48) * (2 * PI) = 1.57079633
   */
  public static final double ROLLER_POSITION_CONVERSION_FACTOR = 1.57079633; // TODO: check belt ratio
  // RPM to inches per second
  public static final double ROLLER_VELOCITY_CONVERSION_FACTOR = ROLLER_POSITION_CONVERSION_FACTOR / 60;

  public static final double ROLLER_kP = 3.596;
  public static final double ROLLER_kI = 0;
  public static final double ROLLER_kD = 0;

}
