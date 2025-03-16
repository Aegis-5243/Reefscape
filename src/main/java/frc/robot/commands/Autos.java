// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.commands.ArmTo.ArmLocation;
import frc.robot.commands.ElevatorTo.ElevatorLocation;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsytem;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.util.LimelightHelpers;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public final class Autos {

	public static ComplexWidget tmp;

	/** Example static factory for an autonomous command. */
	public static Command exampleAuto(DriveSubsystem subsystem) {
		return Commands.sequence(subsystem.exampleMethodCommand(), new DriveCommand(subsystem));
	}

	/**
	 * Command to run SysID routine for drive.
	 * 
	 * <p>
	 * Will disable WPILib motor safety for duration of test.
	 * 
	 * @param subsystem The drive subsystem.
	 * @param type      The type of routine (i.e. qusistatic).
	 * @param dir       The direction to turn the motors.
	 * @return The command to run the routine.
	 */
	public static Command driveSysIdRoutine(DriveSubsystem subsystem, RoutineType type, SysIdRoutine.Direction dir) {
		Command command;
		switch (type) {
			case DYNAMIC:
				command = subsystem.sysId.dynamic(dir);
				break;

			case QUASISTATIC:
				command = subsystem.sysId.quasistatic(dir);
				break;

			default:
				throw new RuntimeException("Supply a valid routine type");
		}
		return new SequentialCommandGroup(
				subsystem.runOnce(() -> {
					subsystem.drive.setSafetyEnabled(false);
				}),
				command,
				subsystem.runOnce(() -> {
					subsystem.drive.setSafetyEnabled(true);
				}));
	}

	/**
	 * @deprecated Use in built functions
	 *             Command to run a SysID routine.
	 * @param routine
	 * @param type
	 * @param dir
	 * @return The command to run the routine.
	 */
	@Deprecated
	public static Command sysIdRoutine(SysIdRoutine routine, RoutineType type, SysIdRoutine.Direction dir) {
		switch (type) {
			case DYNAMIC:
				return routine.dynamic(dir);

			case QUASISTATIC:
				return routine.quasistatic(dir);

			default:
				throw new RuntimeException("Supply a valid routine type");
		}
	}

	/**
	 * Routine type for SysID routines
	 * <p>
	 * Dynamic - Runs the motors at a constant rate
	 * <p>
	 * Quasistaic - Runs motors at an increasing rate
	 */
	public static enum RoutineType {
		DYNAMIC,
		QUASISTATIC
	}

	/**
	 * Test motor by running it at 50% power.
	 * 
	 * @param sub   Subsystem that contains this motor. Output command will require
	 *              this subsystem.
	 * @param motor Motor to run.
	 * @return The command to run the motor
	 */
	public static Command testMotor(Subsystem sub, MotorController motor) {
		return testMotor(sub, motor, 0.5);
	}

	/**
	 * Test motor by running it.
	 * 
	 * @param sub   Subsystem that contains this motor. Output command will require
	 *              this subsystem.
	 * @param motor Motor to run.
	 * @param speed Speed that the motor should run.
	 * @return The command to run the motor
	 */
	public static Command testMotor(Subsystem sub, MotorController motor, double speed) {
		return sub.startEnd(() -> {
			motor.set(speed);
		}, () -> {
			motor.set(0);
		});
	}

	/**
	 * Used to PID tune a motor
	 * 
	 * @param sub      Subsystem that contains this motor. Output command will
	 *                 require this subsystem.
	 * @param motor    Motor to tune.
	 * @param motorPID PID controller of the motor.
	 * @param encoder  Encoder attached to the motor.
	 * @return The command to tune the motor.
	 */
	public static Command tunePID(SubsystemBase sub, MotorController motor, PIDController motorPID, Encoder encoder) {
		return sub.startRun(() -> {
			tmp = Shuffleboard.getTab("pid test").add("PID", motorPID);// .withWidget(BuiltInWidgets.kPIDController);
		}, () -> {
			// motorPID.setP(tmp.getDouble(Constants.BL_kP));
			motor.setVoltage(motorPID.calculate(encoder.getRate()));
			// System.out.println(tmp.getDouble(0));
		});

	}

	public static Command middleStartL4Score(DriveSubsystem m_driveSubsystem, ArmSubsystem m_armSubsystem,
			ElevatorSubsytem m_elevatorSubsytem, RollerSubsystem m_rollerSubsystem) {
		return new SequentialCommandGroup(
				new Wait(5),
				new ParallelCommandGroup(
						new EncoderDrive(m_driveSubsystem, Units.Feet.of(4.5)),
						new SequentialCommandGroup(
								new ArmTo(ArmLocation.DURING_ELEVATOR_MOVEMENT, m_armSubsystem),
								new ElevatorTo(ElevatorLocation.HIGH_CORAL, m_elevatorSubsytem),
								new ArmTo(ArmLocation.HIGH_CORAL, m_armSubsystem))),

				new AutonBringCoralUp(m_rollerSubsystem),
				new Wait(1),
				new EncoderDrive(m_driveSubsystem, Units.Feet.of(2)),
				new Wait(1),
				new Outtake(m_rollerSubsystem),
				new Wait(1));
	}

	public static Command coralmaybe(DriveSubsystem m_driveSubsystem, ArmSubsystem m_armSubsystem,
			ElevatorSubsytem m_elevatorSubsytem, RollerSubsystem m_rollerSubsystem, CameraSubsystem m_cameraSubsystem) {
		return new SequentialCommandGroup(
				new ParallelCommandGroup(
						new EncoderDrive(m_driveSubsystem, Units.Feet.of(8))
				// new SequentialCommandGroup(
				// new ArmTo(ArmLocation.DURING_ELEVATOR_MOVEMENT, m_armSubsystem),
				// new ElevatorTo(ElevatorLocation.HIGH_CORAL, m_elevatorSubsytem),
				// new ArmTo(ArmLocation.HIGH_CORAL, m_armSubsystem)))
				),
				new Wait(5),
				new EncoderTurn(m_driveSubsystem, Units.Degrees.of(-100)),
				new Wait(1),
				// new AutonBringCoralUp(m_rollerSubsystem),
				// new ParallelRaceGroup(
				new EncoderDrive(m_driveSubsystem, Units.Inches.of(64)),
				// new ElevatorCommand(m_elevatorSubsytem)
				// )
				new AlignCoralAuto(m_driveSubsystem, m_cameraSubsystem),
				new EncoderDrive(m_driveSubsystem, Units.Inches.of(6), .15),
				// score
				new EncoderDrive(m_driveSubsystem, Units.Inches.of(-64)));
	}

	public static Command moveForward(double delay, DriveSubsystem m_driveSubsystem) {
		return new SequentialCommandGroup(
				new Wait(delay),
				new EncoderDrive(m_driveSubsystem, Units.Feet.of(4.5)));
	}

	public static Command limlit(DriveSubsystem m_driveSubsystem, ArmSubsystem m_armSubsystem,
			ElevatorSubsytem m_elevatorSubsytem, RollerSubsystem m_rollerSubsystem, CameraSubsystem m_cameraSubsystem) {
		return new SequentialCommandGroup(
				new Wait(1),

				new ParallelCommandGroup(
						new EncoderDrive(m_driveSubsystem, Units.Feet.of(5.25)),
						new SequentialCommandGroup(
								new ArmTo(ArmLocation.DURING_ELEVATOR_MOVEMENT, m_armSubsystem),
								new ElevatorTo(ElevatorLocation.HIGH_CORAL, m_elevatorSubsytem),
								new ArmTo(ArmLocation.HIGH_CORAL, m_armSubsystem))),

				// (new Turn(m_driveSubsystem, m_cameraSubsystem.fieldLayout.getTagPose((int)LimelightHelpers.getFiducialID(Constants.FRONT_LIMELIGHT)).orElse())),
				new ParallelRaceGroup(
						new AlignCoralAuto(m_driveSubsystem, m_cameraSubsystem, Constants.LEFT_CORAL_PIPELINE),
						new ElevatorCommand(m_elevatorSubsytem)),
				new ArmTo(ArmLocation.HIGH_CORAL, m_armSubsystem),

				new AutonBringCoralUp(m_rollerSubsystem),
				new Wait(0.4),
				new TimeDrive(m_driveSubsystem, 2, 0.2),
				new Wait(1),
				new ParallelRaceGroup(
					new Outtake(m_rollerSubsystem),
					new Wait(1)	
				),
				new ParallelCommandGroup(
						new ElevatorCommand(m_elevatorSubsytem),
						new TimeDrive(m_driveSubsystem, 0.5, -0.4))

		// new Wait(3)
		);
	}

	private Autos() {
		throw new UnsupportedOperationException("This is a utility class!");
	}
}
