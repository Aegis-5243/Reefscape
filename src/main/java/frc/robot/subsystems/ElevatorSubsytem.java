// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class ElevatorSubsytem extends SubsystemBase {
	/** Creates a new ExampleSubsystem. */
	private static ElevatorSubsytem instance;

	public SparkMax elevator;
	public SparkMax elevatorMinion;

	public SysIdRoutine sysId;

	public RelativeEncoder elevatorEncoder;
	public RelativeEncoder elevatorMinionEncoder;

	public ElevatorFeedforward elevatorFeedforward;
	public ElevatorFeedforward elevatorMinionFeedforward;

	public SparkClosedLoopController elevatorPIDController;
	public SparkClosedLoopController elevatorMinionPIDController;

	public DigitalInput limitSwitch;

	public double manualSetpoint;
	public boolean done;

	public ElevatorSubsytem() {

		this.elevator = new SparkMax(Constants.ELEVATOR_PRIMARY, MotorType.kBrushless);
		this.elevatorMinion = new SparkMax(Constants.ELEVATOR_SECONDARY, MotorType.kBrushless);

		elevator.configure(
				new SparkMaxConfig().idleMode(IdleMode.kBrake).disableFollowerMode().inverted(true)
						.apply(new SoftLimitConfig().reverseSoftLimit(0))
						/*.apply(new ClosedLoopConfig().p(Constants.ELEVATOR_kP).i(Constants.ELEVATOR_kI)
								.d(Constants.ELEVATOR_kD).feedbackSensor(FeedbackSensor.kPrimaryEncoder)
								.apply(new MAXMotionConfig().maxVelocity(Constants.ELEVATOR_MAX_VELOCITY.in(Units.RPM))
										.maxAcceleration(
												Constants.ELEVATOR_MAX_ACCELERATION.in(Units.RPM.per(Units.Second)))))*/,
				ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		// With own pid
		elevatorMinion.configure(
				new SparkMaxConfig().idleMode(IdleMode.kBrake).disableFollowerMode().inverted(false)
						.apply(new SoftLimitConfig().reverseSoftLimit(0))
						/*.apply(new ClosedLoopConfig().p(Constants.ELEVATOR_MINION_kP).i(Constants.ELEVATOR_MINION_kI)
								.d(Constants.ELEVATOR_MINION_kD).feedbackSensor(FeedbackSensor.kPrimaryEncoder)
								.apply(new MAXMotionConfig().maxVelocity(Constants.ELEVATOR_MAX_VELOCITY.in(Units.RPM))
										.maxAcceleration(
												Constants.ELEVATOR_MAX_ACCELERATION.in(Units.RPM.per(Units.Second)))))*/,
				ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		// Without own pid
		// elevatorMinion.configure(
		// new SparkMaxConfig().idleMode(IdleMode.kBrake).follow(elevator, true),
		// ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		this.elevatorEncoder = elevator.getEncoder();
		this.elevatorMinionEncoder = elevatorMinion.getEncoder();

		// this.elevatorEncoder.setPosition(0);
		// this.elevatorMinionEncoder.setPosition(0);

		this.limitSwitch = new DigitalInput(Constants.ELEVATOR_HALL_EFFECT_PORT);

		this.elevatorFeedforward = new ElevatorFeedforward(Constants.ELEVATOR_kS, Constants.ELEVATOR_kG,
				Constants.ELEVATOR_kV, Constants.ELEVATOR_kA);
		this.elevatorMinionFeedforward = new ElevatorFeedforward(Constants.ELEVATOR_MINION_kS,
				Constants.ELEVATOR_MINION_kG, Constants.ELEVATOR_MINION_kV, Constants.ELEVATOR_MINION_kA);

		this.elevatorPIDController = elevator.getClosedLoopController();
		this.elevatorMinionPIDController = elevatorMinion.getClosedLoopController();

		this.manualSetpoint = 0;

		instance = this;
	}

	public static ElevatorSubsytem getInstance() {
		return instance;
	}

	public void elevator() {
		double speed = ((Constants.primaryStick.getThrottle() + 1) / 2)
				+ ((Constants.secondaryStick.getThrottle() - 1) / 2);

		// System.out.println(speed);
		// System.out.print(elevatorMinionEncoder.getPosition() * Constants.ELEVATOR_HEIGHT_PER_MOTOR_ROT.in(Units.Inches) + ", ");
		// System.out.println(elevatorMinionEncoder.getPosition());

		// replace with applySpeed after proper testing and wiring.
		applySpeed(speed, elevator);
		applySpeed(speed, elevatorMinion);

		// System.out.println(elevator.get());
	}

	public void elevator(double speed) {

		// System.out.println(speed);
		// System.out.print(elevatorMinionEncoder.getPosition() * Constants.ELEVATOR_HEIGHT_PER_MOTOR_ROT.in(Units.Inches) + ", ");
		// System.out.println(elevatorMinionEncoder.getPosition());

		// replace with applySpeed after proper testing and wiring.
		applySpeed(speed, elevator);
		applySpeed(speed, elevatorMinion);

		// System.out.println(elevator.get());
	}

	public void applySpeed(double speed, SparkMax motor) {
		if (!limitSwitch.get()) {
			motor.getEncoder().setPosition(0);
		}

		speed = (elevatorEncoder.getPosition() <= 0 && speed < 0) || (elevatorEncoder.getPosition()
				* Constants.ELEVATOR_HEIGHT_PER_MOTOR_ROT.in(Units.Meters) >= Constants.ELEVATOR_MAX_HEIGHT
						.minus(Units.Inches.of(6)).in(Units.Meters)
				&& speed > 0) ? 0 : speed;

		motor.set(speed);
	}

	public void setTargetPositionPID(double rotations) {
		elevatorPIDController.setReference(rotations, ControlType.kMAXMotionPositionControl);
		elevatorMinionPIDController.setReference(rotations, ControlType.kMAXMotionPositionControl);
	}

	public void setTargetPositionMan(double rotations) {
		manualSetpoint = rotations;
		done = false;
	}

	public void runToSetpoint() {
		// System.out.println(elevatorEncoder.getPosition());
		if (elevatorEncoder.getPosition() < manualSetpoint - Constants.ELEVATOR_MAN_TOLERANCE || elevatorEncoder.getPosition() > manualSetpoint + Constants.ELEVATOR_MAN_TOLERANCE) {
			double diff = (manualSetpoint - elevatorEncoder.getPosition()) / (manualSetpoint * 6);
			diff = diff > 1 ? 1 : diff;
			diff = diff < -1 ? -1 : diff;
			diff = Math.abs(diff) < .3 ? Math.signum(diff) * .3 : diff;
			applySpeed(diff, elevator);
			System.out.println("in");
		} else {
			done = true;
			System.out.println("out");
		}
		if (elevatorMinionEncoder.getPosition() < manualSetpoint - Constants.ELEVATOR_MAN_TOLERANCE || elevatorMinionEncoder.getPosition() > manualSetpoint + Constants.ELEVATOR_MAN_TOLERANCE) {
			double diff = (manualSetpoint - elevatorMinionEncoder.getPosition()) / (manualSetpoint * 6);
			diff = diff > 1 ? 1 : diff;
			diff = diff < -1 ? -1 : diff;
			diff = Math.abs(diff) < .3 ? Math.signum(diff) * .3 : diff;
			applySpeed(diff, elevatorMinion);
		}
	}

	public boolean isStill() {
		return elevatorEncoder.getVelocity() == 0 && elevatorMinionEncoder.getVelocity() == 0;
	}

	public boolean isDone() {
		return done;
	}

	/**
	 * Example command factory method.
	 *
	 * @return a command
	 */
	public Command exampleMethodCommand() {
		// Inline construction of command goes here.
		// Subsystem::RunOnce implicitly requires `this` subsystem.
		return runOnce(
				() -> {
					/* one-time action goes here */
				});
	}

	/**
	 * An example method querying a boolean state of the subsystem (for example, a
	 * digital sensor).
	 *
	 * @return value of some boolean subsystem state, such as a digital sensor.
	 */
	public boolean exampleCondition() {
		// Query some boolean state, such as a digital sensor.
		return false;
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run

	}

	@Override
	public void simulationPeriodic() {
		// This method will be called once per scheduler run during simulation
	}
}
