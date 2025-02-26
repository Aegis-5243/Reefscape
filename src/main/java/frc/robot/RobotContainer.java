// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.ArmCommand;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.ElevatorTo;
import frc.robot.commands.Wait;
import frc.robot.commands.Autos.RoutineType;
import frc.robot.commands.ElevatorTo.ElevatorLocation;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsytem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
	// The robot's subsystems and commands are defined here...
	private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
	private final ElevatorSubsytem m_elevatorSubsytem = new ElevatorSubsytem();
	private final ArmSubsystem m_armSubsystem = new ArmSubsystem();

	private final DriveCommand m_driveCommand = new DriveCommand(m_driveSubsystem);
	private final ElevatorCommand m_elevatorCommand = new ElevatorCommand(m_elevatorSubsytem);
	private final ArmCommand m_armCommand = new ArmCommand(m_armSubsystem);
	// Replace with CommandPS4Controller or CommandJoystick if needed
	// private final CommandXboxController m_driverController = new CommandXboxController(
	// 		OperatorConstants.kDriverControllerPort);

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		m_driveSubsystem.setDefaultCommand(m_driveCommand);
		m_elevatorSubsytem.setDefaultCommand(m_elevatorCommand);
		m_armSubsystem.setDefaultCommand(m_armCommand);

		// Configure the trigger bindings
		configureBindings();
	}

	/**
	 * Use this method to define your trigger->command mappings. Triggers can be
	 * created via the
	 * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
	 * an arbitrary
	 * predicate, or via the named factories in {@link
	 * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
	 * {@link
	 * CommandXboxController
	 * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
	 * PS4} controllers or
	 * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
	 * joysticks}.
	 */
	private void configureBindings() {
		// Schedule `ExampleCommand` when `exampleCondition` changes to `true`
		// new Trigger(m_exampleSubsystem::exampleCondition)
		// 		.onTrue(new DriveCommand(m_exampleSubsystem));

		// Schedule `exampleMethodCommand` when the Xbox controller's B button is
		// pressed,
		// cancelling on release.
		// m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
		new JoystickButton(Constants.primaryStick, 3).onTrue(new ElevatorTo(ElevatorLocation.THROUGH, m_elevatorSubsytem));
		new JoystickButton(Constants.primaryStick, 5).onTrue(new ElevatorTo(ElevatorLocation.LOW_CORAL, m_elevatorSubsytem));
		new JoystickButton(Constants.primaryStick, 4).onTrue(new ElevatorTo(ElevatorLocation.MID_CORAL, m_elevatorSubsytem));
		// new JoystickButton(Constants.primaryStick,6).onTrue(new ElevatorTo(ElevatorLocation.LOW_CORAL, m_elevatorSubsytem));
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		// An example command will be run in autonomous
		// return new SequentialCommandGroup(Autos.sysIdRoutine(m_driveSubsystem, RoutineType.QUASISTATIC, Direction.kForward), new Wait(5), Autos.sysIdRoutine(m_driveSubsystem, RoutineType.DYNAMIC, Direction.kForward), new Wait(5), Autos.sysIdRoutine(m_driveSubsystem, RoutineType.QUASISTATIC, Direction.kReverse), new Wait(5), Autos.sysIdRoutine(m_driveSubsystem, RoutineType.DYNAMIC, Direction.kReverse)); 
		// return Autos.testMotor(m_driveSubsystem, m_driveSubsystem.bl);
		return Autos.testPID(m_driveSubsystem, m_driveSubsystem.bl, m_driveSubsystem.blPID, m_driveSubsystem.blEncoder);
		// return Autos.sysIdRoutine(m_driveSubsystem, RoutineType.DYNAMIC);
	}
}
