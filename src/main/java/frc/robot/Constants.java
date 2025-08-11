// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.units.Units;
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

    /* Mecanum Drive */

    public static final int FL = 12;
    public static final int FR = 11;
    public static final int BL = 14;
    public static final int BR = 13;

    public static final int[] FL_ENCODER_PORTS = { 7, 6 };
    public static final int[] FR_ENCODER_PORTS = { 1, 0 };
    public static final int[] BL_ENCODER_PORTS = { 5, 4 };
    public static final int[] BR_ENCODER_PORTS = { 3, 2 };

    /*
     * Mecanum conversions
     * 
     * Gear ratio of 12.75 creates a difference in alternate encoders vs motor
     * encoders
     * alternate = base / 12.75
     * 
     * 
     */

    public static final double MECANUM_WHEEL_DIAMETER = 6;

    /** Rotations to meters of mecanum alternate encoders */
    public static final double MECANUM_ALTERNATE_POSITION_CONVERSION_FACTOR = Math.PI *
            Units.Meters.convertFrom(Constants.MECANUM_WHEEL_DIAMETER, Units.Inches);
    /** RPM to meters per second of mecanum alternate encoders */
    public static final double MECANUM_ALTERNATE_VELOCITY_CONVERSION_FACTOR = Math.PI *
            Units.Meters.convertFrom(Constants.MECANUM_WHEEL_DIAMETER, Units.Inches) / 60;

    /** Rotations to meters of mecanum encoders */
    public static final double MECANUM_POSITION_CONVERSION_FACTOR = MECANUM_ALTERNATE_POSITION_CONVERSION_FACTOR
            / 12.76;
    /** RPM to meters per second of mecanum encoders */
    public static final double MECANUM_VELOCITY_CONVERSION_FACTOR = MECANUM_ALTERNATE_VELOCITY_CONVERSION_FACTOR
            / 12.76;

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

    public static final Distance TRACK_WIDTH = Units.Inches.of(22.25);
    public static final Distance TRACK_HEIGHT = Units.Inches.of(20.3599);

    /** Maximum drive speed in meters per second. */
    public static final double DRIVE_MAX_SPEED = 1;
    /** Maximum drive acceleration in meters per second squared. */
    public static final double DRIVE_MAX_ACCELERATION = 1;
    /** Maximum drive angular speed in radians per second. */
    public static final double DRIVE_MAX_ANGULAR_SPEED = 3.14;
    /** Maximum drive angular acceleration in radians per second squared. */
    public static final double DRIVE_MAX_ANGULAR_ACCELERATION = 3.14;

    /* Drive PID constants (meters)*/
    public static final double DRIVE_kP = 5.0;
    public static final double DRIVE_kI = 0;
    public static final double DRIVE_kD = 1.0;

    /* Drive rotation PID constants (degrees) */
    public static final double DRIVE_ROTATE_kP = 0.08;
    public static final double DRIVE_ROTATE_kI = 0;
    public static final double DRIVE_ROTATE_kD = 0.009;

    public static final double RIO_CONTROL_LOOP = 0.1;

    /* Vision */
    /** The name of the limelight on the bottom front of the bot */
    public static final String FRONT_LIMELIGHT = "limelight";

    /* Elevator */

    /*
     * Elevator conversion factors
     * Gear reduction is 1:20
     * Chain pitch is 0.25 inches
     * Chain gear is 22t
     * Inner stage moves x2 the chain speed
     * = 0.25 * 22 * 1/20 * 2 = 0.55
     */
    /** Rotations to inches for elevator motor encoders */
    public static final double ELEVATOR_POSITION_CONVERSION_FACTOR = 0.55;
    /** RPM to inches per second for elevator motor encoders */
    public static final double ELEVATOR_VELOCITY_CONVERSION_FACTOR = ELEVATOR_POSITION_CONVERSION_FACTOR / 60;

    /** Maximum velocity of elevator in inches per second */
    public static final double ELEVATOR_MAX_VELOCITY = 12;

    public static final double ELEVATOR_kP = 0.1;
    public static final double ELEVATOR_kI = 0;
    public static final double ELEVATOR_kD = 0.02;

    /** Maximum height of elevator in inches, used for soft limit */
    public static final double ELEVATOR_MAX_HEIGHT = 50.5;

    public static final int ELEVATOR_LEFT_MOTOR_PORT = 21;
    public static final int ELEVATOR_RIGHT_MOTOR_PORT = 22;

    public static final int ELEVATOR_HALL_EFFECT_PORT = 8;

    public static final double ELEVATOR_MAN_TOLERANCE = .5;

    public static final double ELEVATOR_FF_VOLTAGE = 0.385;

    public static final double ELEVATOR_STALL_CURRENT = 30.0;

    /* Arm */
    public static final int ARM_MOTOR_PORT = 31;

    /*
     * Arm conversion factors
     * Gear ratio: 1:18
     * Rotations to degrees: 360
     * = 360 / 18 = 20
     */
    /** Rotations to degrees for arm motor encoder */
    public static final double ARM_POSITION_CONVERSION_FACTOR = 20.0;
    /** RPM to degrees per second for arm motor encoder */
    public static final double ARM_VELOCITY_CONVERSION_FACTOR = ARM_POSITION_CONVERSION_FACTOR / 60;

    /** Maximum position of arm in degrees, used for soft limit */
    public static final double ARM_MAX_POS = 180;
    /** Minimum position of arm in degrees, used for soft limit */
    public static final double ARM_MIN_POS = 17;

    /** Maximum velocity of arm in maxMotion mode in degrees per second */
    public static final double ARM_MAX_VELOCITY = 90;
    /**
     * Maximum acceleraton of arm in maxMotion mode in degrees per second squared
     */
    public static final double ARM_MAX_ACCELERATION = 180;
    
    public static final double ARM_kP = 0.03;
    public static final double ARM_kI = 0;
    public static final double ARM_kD = 0.05;
    
    public static final int ARM_LIMIT_SWITCH_PORT = 9;
    
    /* Roller */
    public static final int ROLLER_PORT = 32;
    
    public static final int ROLLER_TIME_OF_FLIGHT_1_PORT = 33;
    public static final int ROLLER_TIME_OF_FLIGHT_2_PORT = 34;

    /*
     * Roller conversion factors
     * Belt ratio: 12 / 30
     * Roller diameter: 2 inches
     * = (12 / 30) * (2 * PI)
     */
    /** Rotations to inches for intake roller encoders */
    public static final double ROLLER_POSITION_CONVERSION_FACTOR = 12. / 30 * 2 * Math.PI;
    /** RPM to inches per second for intake roller encoders */
    public static final double ROLLER_VELOCITY_CONVERSION_FACTOR = ROLLER_POSITION_CONVERSION_FACTOR / 60;

    public static final double ROLLER_kP = 0.13;
    public static final double ROLLER_kI = 0;
    public static final double ROLLER_kD = 0.05;

    /** Coral detection threshold of the time of flight sensor located at the start of the intake in mm */
    public static final double ROLLER_CORAL_DETECTION_THRESHOLD_1 = 70;
    /** Coral detection threshold of the time of flight sensor located at the end of the intake in mm */
    public static final double ROLLER_CORAL_DETECTION_THRESHOLD_2 = 70;

}
