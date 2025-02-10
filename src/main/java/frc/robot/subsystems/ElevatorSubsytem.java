// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.playingwithfusion.CANVenom;
import com.playingwithfusion.CANVenom.BrakeCoastMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.libs.VelocityEncoder;
import frc.robot.Constants;
import frc.robot.util.Utilities;

public class ElevatorSubsytem extends SubsystemBase {
    /** Creates a new ExampleSubsystem. */
    private static ElevatorSubsytem instance;

    public SparkMax elevator;
    public SparkMax elevatorMinion;

    public SysIdRoutine sysId;

    public SparkAbsoluteEncoder elevatorEncoder;
    
    public ElevatorSubsytem() {
        
        this.elevator = new SparkMax(Constants.ELEVATOR_PRIMARY, MotorType.kBrushless);
        this.elevatorMinion = new SparkMax(Constants.ELEVATOR_SECONDARY, MotorType.kBrushless);

        elevator.configure(new SparkMaxConfig().idleMode(IdleMode.kBrake), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        elevatorMinion.configure(new SparkMaxConfig().idleMode(IdleMode.kBrake).follow(elevator, true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        this.elevatorEncoder = elevator.getAbsoluteEncoder();

		instance = this;

	}

	public static ElevatorSubsytem getInstance() {
		return instance;
	}


	public void elevator() {
		elevator.set(((Constants.primaryStick.getThrottle() + 1) / 2) + ((Constants.secondaryStick.getThrottle() - 1) / 2));
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
