// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.AlignAlgae;
import frc.robot.commands.AlignCoral;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.ArmTo;
import frc.robot.commands.AutonBringCoralUp;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.ElevatorDown;
import frc.robot.commands.ElevatorTo;
import frc.robot.commands.EncoderDrive;
import frc.robot.commands.Intake;
import frc.robot.commands.Outtake;
import frc.robot.commands.RollerCommand;
import frc.robot.commands.ArmTo.ArmLocation;
import frc.robot.commands.ElevatorTo.ElevatorLocation;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsytem;
import frc.robot.subsystems.RollerSubsystem;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
	private final RollerSubsystem m_rollerSubsystem = new RollerSubsystem();
	private final CameraSubsystem m_cameraSubsystem = new CameraSubsystem();

	private final DriveCommand m_driveCommand = new DriveCommand(m_driveSubsystem);
	private final ElevatorCommand m_elevatorCommand = new ElevatorCommand(m_elevatorSubsytem);
	private final ArmCommand m_armCommand = new ArmCommand(m_armSubsystem);
	private final RollerCommand m_rollerCommand = new RollerCommand(m_rollerSubsystem);

	private final SendableChooser<Command> m_chooser = new SendableChooser<>();
	// Replace with CommandPS4Controller or CommandJoystick if needed
	// private final CommandXboxController m_driverController = new
	// CommandXboxController(
	// OperatorConstants.kDriverControllerPort);

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		m_driveSubsystem.setDefaultCommand(m_driveCommand);
		m_elevatorSubsytem.setDefaultCommand(m_elevatorCommand);
		m_armSubsystem.setDefaultCommand(m_armCommand);
		m_rollerSubsystem.setDefaultCommand(m_rollerCommand);

		m_chooser.setDefaultOption("Middle position L4 coral no limelight", Autos.middleStartL4Score(m_driveSubsystem, m_armSubsystem, m_elevatorSubsytem, m_rollerSubsystem));
		m_chooser.addOption("Move forward", Autos.moveForward(5, m_driveSubsystem));
		m_chooser.addOption("limlit", Autos.limlit(m_driveSubsystem, m_armSubsystem, m_elevatorSubsytem, m_rollerSubsystem, m_cameraSubsystem));
		m_chooser.addOption("DO NOT USE IN COMP", new EncoderDrive(m_driveSubsystem, Units.Meters.of(300), 1));
		SmartDashboard.putData("auton chooser", m_chooser);

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
		// .onTrue(new DriveCommand(m_exampleSubsystem));

		// Schedule `exampleMethodCommand` when the Xbox controller's B button is
		// pressed,
		// cancelling on release.
		// m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

		NamedCommands.registerCommand("Mechanism to Intake", new SequentialCommandGroup(
			new ArmTo(ArmLocation.DURING_ELEVATOR_MOVEMENT, m_armSubsystem),
			new ElevatorTo(ElevatorLocation.INTAKE, m_elevatorSubsytem),
			new ArmTo(ArmLocation.INTAKE, m_armSubsystem)));

			
		NamedCommands.registerCommand("Mechanism to High Coral", new SequentialCommandGroup(
			new ArmTo(ArmLocation.DURING_ELEVATOR_MOVEMENT, m_armSubsystem),
			new ElevatorTo(ElevatorLocation.HIGH_CORAL, m_elevatorSubsytem),
			new ArmTo(ArmLocation.HIGH_CORAL, m_armSubsystem)));

			
		NamedCommands.registerCommand("Mechanism to Low Coral", new SequentialCommandGroup(
			new ArmTo(ArmLocation.DURING_ELEVATOR_MOVEMENT, m_armSubsystem),
			new ElevatorTo(ElevatorLocation.LOW_CORAL, m_elevatorSubsytem),
			new ArmTo(ArmLocation.LOW_CORAL, m_armSubsystem)));

			
		NamedCommands.registerCommand("Intake", new Intake(m_rollerSubsystem));

		NamedCommands.registerCommand("Outtake", new Outtake(m_rollerSubsystem));

		NamedCommands.registerCommand("Coral Up", new AutonBringCoralUp(m_rollerSubsystem));

		NamedCommands.registerCommand("Elevator Command", m_elevatorCommand);

		new JoystickButton(Constants.secondaryStick, 3).onTrue(new SequentialCommandGroup(
				new ArmTo(ArmLocation.DURING_ELEVATOR_MOVEMENT, m_armSubsystem),
				new ElevatorTo(ElevatorLocation.THROUGH, m_elevatorSubsytem),
				new ArmTo(ArmLocation.THROUGH, m_armSubsystem)));

		new JoystickButton(Constants.secondaryStick, 4).onTrue(new SequentialCommandGroup(
				new ArmTo(ArmLocation.DURING_ELEVATOR_MOVEMENT, m_armSubsystem),
				new ElevatorTo(ElevatorLocation.LOW_CORAL, m_elevatorSubsytem),
				new ArmTo(ArmLocation.LOW_CORAL, m_armSubsystem)));

		new JoystickButton(Constants.secondaryStick, 5).onTrue(new SequentialCommandGroup(
				new ArmTo(ArmLocation.DURING_ELEVATOR_MOVEMENT, m_armSubsystem),
				new ElevatorTo(ElevatorLocation.MID_CORAL, m_elevatorSubsytem),
				new ArmTo(ArmLocation.MID_CORAL, m_armSubsystem)));

		new JoystickButton(Constants.secondaryStick, 6).onTrue(new SequentialCommandGroup(
				new ArmTo(ArmLocation.DURING_ELEVATOR_MOVEMENT, m_armSubsystem),
				new ElevatorTo(ElevatorLocation.INTAKE, m_elevatorSubsytem),
				new ArmTo(ArmLocation.INTAKE, m_armSubsystem)));

		new JoystickButton(Constants.primaryStick, 4).onTrue(new SequentialCommandGroup(
				new ArmTo(ArmLocation.DURING_ELEVATOR_MOVEMENT, m_armSubsystem),
				new ElevatorTo(ElevatorLocation.HIGH_CORAL, m_elevatorSubsytem),
				new ArmTo(ArmLocation.HIGH_CORAL, m_armSubsystem)));

		new JoystickButton(Constants.secondaryStick, 2).onTrue(new Intake(m_rollerSubsystem));
		new JoystickButton(Constants.secondaryStick, 1).onTrue(new Outtake(m_rollerSubsystem));

		new JoystickButton(Constants.primaryStick, 1).whileTrue(m_rollerSubsystem.startRun(() -> {
		}, () -> {
			m_rollerSubsystem.roller.set(.05);
			m_rollerSubsystem.rollerEncoder.setPosition(0);
		}));

		new JoystickButton(Constants.primaryStick, 5).whileTrue(new AlignCoral(m_driveSubsystem, m_cameraSubsystem, Constants.LEFT_CORAL_PIPELINE));
		new JoystickButton(Constants.primaryStick, 6).whileTrue(new AlignCoral(m_driveSubsystem, m_cameraSubsystem, 1));

		new JoystickButton(Constants.primaryStick, 3).whileTrue(new ElevatorDown(m_elevatorSubsytem));
		
		new JoystickButton(Constants.primaryStick, 11).whileTrue(new SequentialCommandGroup(
			// new ArmTo(ArmLocation.DURING_ELEVATOR_MOVEMENT, m_armSubsystem),
			new ArmTo(Units.Degrees.of(120), m_armSubsystem),
			new ElevatorTo(Units.Inches.of(10), m_elevatorSubsytem),
			new ParallelCommandGroup(
				m_rollerSubsystem.startEnd(() -> {m_rollerSubsystem.roller.set(.5);}, () -> {m_rollerSubsystem.roller.set(0);m_rollerSubsystem.rollerEncoder.setPosition(0);}),
				new AlignAlgae(m_driveSubsystem)
			)
		));
		
		// new JoystickButton(Constants.primaryStick, 12).whileTrue(new SequentialCommandGroup(
		// 	new ArmTo(Units.Degrees.of(120), m_armSubsystem),
		// 	new ParallelCommandGroup(
		// 		m_rollerSubsystem.startEnd(() -> {m_rollerSubsystem.roller.set(.25);}, () -> {m_rollerSubsystem.roller.set(0);m_rollerSubsystem.rollerEncoder.setPosition(0);}),
		// 		m_driveSubsystem.startEnd(() -> {m_driveSubsystem.mechDrive(.25, 0, 0);}, () -> {m_driveSubsystem.mechDrive(0, 0, 0);}).repeatedly()
		// 	)
		// 	// new ParallelCommandGroup(
		// 	// 	m_rollerSubsystem.startEnd(() -> (m_rollerSubsystem.roller.set(-0.5)), null)
		// 	// )

		// ));
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		// An example command will be run in autonomous
		return m_chooser.getSelected();
	}
}
