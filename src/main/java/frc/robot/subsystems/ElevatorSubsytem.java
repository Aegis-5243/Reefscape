// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DigitalInput;
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

	public ElevatorFeedforward elevatorFeedforward;
	public ElevatorFeedforward elevatorMinionFeedforward;

	public PIDController elevatorPIDController;
	public PIDController elevatorMinionPIDController;

	public DigitalInput limitSwitch;

	public ElevatorSubsytem() {

		this.elevator = new SparkMax(Constants.ELEVATOR_PRIMARY, MotorType.kBrushless);
		this.elevatorMinion = new SparkMax(Constants.ELEVATOR_SECONDARY, MotorType.kBrushless);

		elevator.configure(
				new SparkMaxConfig().idleMode(IdleMode.kBrake).apply(new SoftLimitConfig().reverseSoftLimit(0)),
				ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		elevatorMinion.configure(
				new SparkMaxConfig().idleMode(IdleMode.kBrake).follow(elevator, true)
						.apply(new SoftLimitConfig().reverseSoftLimit(0)),
				ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		this.elevatorEncoder = elevator.getEncoder();

		this.limitSwitch = new DigitalInput(Constants.ELEVATOR_HALL_EFFECT_PORT);

		this.elevatorFeedforward = new ElevatorFeedforward(Constants.ELEVATOR_kS, Constants.ELEVATOR_kG,
				Constants.ELEVATOR_kV, Constants.ELEVATOR_kA);
		this.elevatorMinionFeedforward = new ElevatorFeedforward(Constants.ELEVATOR_MINION_kS,
				Constants.ELEVATOR_MINION_kG, Constants.ELEVATOR_MINION_kV, Constants.ELEVATOR_MINION_kA);

		this.elevatorPIDController = new PIDController(Constants.ELEVATOR_kP, Constants.ELEVATOR_kI,
				Constants.ELEVATOR_kD);
		this.elevatorMinionPIDController = new PIDController(Constants.ELEVATOR_MINION_kP, Constants.ELEVATOR_MINION_kI,
				Constants.ELEVATOR_MINION_kD);

		instance = this;

	}

	public static ElevatorSubsytem getInstance() {
		return instance;
	}

	public void elevator() {
		double speed = ((Constants.primaryStick.getThrottle() + 1) / 2)
				+ ((Constants.secondaryStick.getThrottle() - 1) / 2);

		System.out.println(speed);

		if (limitSwitch.get()) {
			elevatorEncoder.setPosition(0);
		}

		speed = (elevatorEncoder.getPosition() <= 0 && speed < 0) || (elevatorEncoder.getPosition()
				* Constants.ELEVATOR_HEIGHT_PER_MOTOR_ROT.in(Units.Inches) >= Constants.ELEVATOR_MAX_HEIGHT
						.minus(Units.Inches.of(6)).in(Units.Inches)
				&& speed > 0) ? 0 : speed;

		elevator.set(speed);
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
