// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.AlignAlgae;
import frc.robot.commands.AlignCoral;
import frc.robot.commands.AlignCoralAuto;
import frc.robot.commands.ArmCommand;
import frc.robot.commands.ArmTo;
import frc.robot.commands.AutonBringCoralUp;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.ElevatorDown;
import frc.robot.commands.ElevatorTo;
import frc.robot.commands.EncoderDrive;
import frc.robot.commands.EncoderTurn;
import frc.robot.commands.Intake;
import frc.robot.commands.Outtake;
import frc.robot.commands.RollerCommand;
import frc.robot.commands.Turn;
import frc.robot.commands.Wait;
import frc.robot.commands.ArmToWPI;
import frc.robot.commands.Autos.RoutineType;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsytem;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.util.Utilities.ArmLocation;
import frc.robot.util.Utilities.ElevatorLocation;

import org.opencv.core.Mat;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
	private final RollerSubsystem m_rollerSubsystem = new RollerSubsystem();
	private final CameraSubsystem m_cameraSubsystem = new CameraSubsystem();

	private final DriveCommand m_driveCommand = new DriveCommand(m_driveSubsystem);
	private final ElevatorCommand m_elevatorCommand = new ElevatorCommand(m_elevatorSubsytem);
	private final ArmCommand m_armCommand = new ArmCommand(m_armSubsystem);
	private final RollerCommand m_rollerCommand = new RollerCommand(m_rollerSubsystem);

	private final SendableChooser<Command> m_chooser;
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

		CommandScheduler.getInstance().registerSubsystem(m_cameraSubsystem);

		m_chooser = AutoBuilder.buildAutoChooser();
		m_chooser.setDefaultOption("limlit", Autos.limlit(m_driveSubsystem, m_armSubsystem, m_elevatorSubsytem,
			m_rollerSubsystem, m_cameraSubsystem));
		m_chooser.addOption("Move forward", Autos.moveForward(5, m_driveSubsystem));
		m_chooser.addOption("Middle position L4 coral no limelight",
				Autos.middleStartL4Score(m_driveSubsystem, m_armSubsystem, m_elevatorSubsytem, m_rollerSubsystem));
		m_chooser.addOption("DO NOT USE IN COMP", Autos.coralmaybe(m_driveSubsystem, m_armSubsystem, m_elevatorSubsytem,
				m_rollerSubsystem, m_cameraSubsystem));

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

		SmartDashboard.putNumber("bl - kV", Constants.BL_kV);
		SmartDashboard.putNumber("br - kV", Constants.BR_kV);
		SmartDashboard.putNumber("fl - kV", Constants.FL_kV);
		SmartDashboard.putNumber("fr - kV", Constants.FR_kV);

		SmartDashboard.putNumber("chassisSpeed", 0);
		SmartDashboard.putNumber("theta", 0);

		NamedCommands.registerCommand("Mechanism to Intake", new SequentialCommandGroup(
				new ArmToWPI(ArmLocation.DURING_ELEVATOR_MOVEMENT, m_armSubsystem),
				new ElevatorTo(ElevatorLocation.INTAKE, m_elevatorSubsytem),
				new ArmToWPI(ArmLocation.INTAKE, m_armSubsystem)));

		NamedCommands.registerCommand("Mechanism to High Coral", new SequentialCommandGroup(
				new ArmToWPI(ArmLocation.DURING_ELEVATOR_MOVEMENT, m_armSubsystem),
				new ElevatorTo(ElevatorLocation.HIGH_CORAL, m_elevatorSubsytem),
				new ArmToWPI(ArmLocation.HIGH_CORAL, m_armSubsystem)));

		NamedCommands.registerCommand("Mechanism to Low Coral", new SequentialCommandGroup(
				new ArmToWPI(ArmLocation.DURING_ELEVATOR_MOVEMENT, m_armSubsystem),
				new ElevatorTo(ElevatorLocation.LOW_CORAL, m_elevatorSubsytem),
				new ArmToWPI(ArmLocation.LOW_CORAL, m_armSubsystem)));

		NamedCommands.registerCommand("Intake", new Intake(m_rollerSubsystem));

		NamedCommands.registerCommand("Outtake", new Outtake(m_rollerSubsystem));

		NamedCommands.registerCommand("Coral Up", new AutonBringCoralUp(m_rollerSubsystem));

		NamedCommands.registerCommand("Elevator Command", m_elevatorCommand);
		
		// Arm to L1
		new JoystickButton(Constants.secondaryStick, 3).onTrue(new SequentialCommandGroup(
				new ArmToWPI(ArmLocation.DURING_ELEVATOR_MOVEMENT, m_armSubsystem),
				new ElevatorTo(ElevatorLocation.THROUGH, m_elevatorSubsytem),
				new ArmToWPI(ArmLocation.THROUGH, m_armSubsystem)
				)
				);

		// Arm to L2
		new JoystickButton(Constants.secondaryStick, 4).onTrue(new SequentialCommandGroup(
				new ArmToWPI(ArmLocation.DURING_ELEVATOR_MOVEMENT, m_armSubsystem),
				new ElevatorTo(ElevatorLocation.LOW_CORAL, m_elevatorSubsytem),
				new ArmToWPI(ArmLocation.LOW_CORAL, m_armSubsystem)));

		// Arm to L3
		new JoystickButton(Constants.secondaryStick, 5).onTrue(new SequentialCommandGroup(
				new ArmToWPI(ArmLocation.DURING_ELEVATOR_MOVEMENT, m_armSubsystem),
				new ElevatorTo(ElevatorLocation.MID_CORAL, m_elevatorSubsytem),
				new ArmToWPI(ArmLocation.MID_CORAL, m_armSubsystem)));

		// Arm to intake pos
		new JoystickButton(Constants.secondaryStick, 6).onTrue(new SequentialCommandGroup(
				new ArmToWPI(ArmLocation.DURING_ELEVATOR_MOVEMENT, m_armSubsystem),
				new ElevatorTo(ElevatorLocation.INTAKE, m_elevatorSubsytem),
				new ArmToWPI(ArmLocation.INTAKE, m_armSubsystem)));

		// Arm to L4
		new JoystickButton(Constants.primaryStick, 4).onTrue(new SequentialCommandGroup(
				new ArmToWPI(ArmLocation.DURING_ELEVATOR_MOVEMENT, m_armSubsystem),
				new ElevatorTo(ElevatorLocation.HIGH_CORAL, m_elevatorSubsytem),
				new ArmToWPI(ArmLocation.HIGH_CORAL, m_armSubsystem)));

		// Intake coral
		new JoystickButton(Constants.secondaryStick, 2).onTrue(new Intake(m_rollerSubsystem));

		// Outtake coral
		new JoystickButton(Constants.secondaryStick, 1).onTrue(new Outtake(m_rollerSubsystem));
		// new JoystickButton(Constants.secondaryStick,
		// 1).whileTrue(m_rollerSubsystem.startRun(() -> {
		// }, () -> {
		// m_rollerSubsystem.roller.set(-.2);
		// m_rollerSubsystem.rollerEncoder.setPosition(0);
		// }));

		// Retract coral
		new JoystickButton(Constants.primaryStick, 1).whileTrue(m_rollerSubsystem.startRun(() -> {
		}, () -> {
			m_rollerSubsystem.roller.set(.1);
			m_rollerSubsystem.rollerEncoder.setPosition(0);
		}));

		// Align and move toward left coral
		new JoystickButton(Constants.primaryStick, 5)
				.whileTrue(new AlignCoral(m_driveSubsystem, m_cameraSubsystem, Constants.LEFT_CORAL_PIPELINE));
		// Align and move toward right coral
		new JoystickButton(Constants.primaryStick, 6)
				.whileTrue(new AlignCoral(m_driveSubsystem, m_cameraSubsystem, Constants.RIGHT_CORAL_PIPELINE));

		// Force elevator down
		new JoystickButton(Constants.primaryStick, 3).whileTrue(new ElevatorDown(m_elevatorSubsytem));

		// Align low algae
		new JoystickButton(Constants.primaryStick, 11).whileTrue(new SequentialCommandGroup(
				// new ArmToWPI(ArmLocation.DURING_ELEVATOR_MOVEMENT, m_armSubsystem),
				new ArmToWPI(Units.Degrees.of(120), m_armSubsystem),
				new ElevatorTo(Units.Inches.of(9.5), m_elevatorSubsytem),
				new ParallelCommandGroup(
						m_rollerSubsystem.startEnd(() -> {
							m_rollerSubsystem.roller.set(.5);
						}, () -> {
							m_rollerSubsystem.roller.set(0);
							m_rollerSubsystem.rollerEncoder.setPosition(0);
						}),
						new AlignAlgae(m_driveSubsystem))));
		// Align high algae
		new JoystickButton(Constants.primaryStick, 12).whileTrue(new SequentialCommandGroup(
				// new ArmToWPI(ArmLocation.DURING_ELEVATOR_MOVEMENT, m_armSubsystem),
				new ArmToWPI(Units.Degrees.of(120), m_armSubsystem),
				new ElevatorTo(Units.Inches.of(26), m_elevatorSubsytem),
				new ParallelCommandGroup(
						m_rollerSubsystem.startEnd(() -> {
							m_rollerSubsystem.roller.set(.5);
						}, () -> {
							m_rollerSubsystem.roller.set(0);
							m_rollerSubsystem.rollerEncoder.setPosition(0);
						}),
						new AlignAlgae(m_driveSubsystem))));
		
		// // EC retract coral
		// new JoystickButton(Constants.emergencyController, XboxController.Button.kY.value).whileTrue(
		// 		m_rollerSubsystem.startRun(() -> {
		// 		},
		// 				() -> {
		// 					m_rollerSubsystem.roller.set(.1);
		// 					m_rollerSubsystem.rollerEncoder.setPosition(0);
		// 				}));
		
		// // EC expel coral
		// new JoystickButton(Constants.emergencyController, XboxController.Button.kA.value).whileTrue(
		// 		m_rollerSubsystem.startRun(() -> {
		// 		},
		// 				() -> {
		// 					m_rollerSubsystem.roller.set(-.1);
		// 					m_rollerSubsystem.rollerEncoder.setPosition(0);
		// 				}));

		// // Cancel arm and elevator commands
		// new JoystickButton(Constants.emergencyController, XboxController.Button.kB.value).whileTrue(
		// 		new ParallelCommandGroup(
		// 				m_elevatorSubsytem.runOnce(() -> {
		// 					m_elevatorSubsytem.elevator(0);
		// 				}),
		// 				m_armSubsystem.runOnce(() -> {
		// 					m_armSubsystem.setpoint = m_armSubsystem.armEncoder.getPosition();
		// 				})));
		
		// // EC force elevator down
		// new JoystickButton(Constants.emergencyController, XboxController.Button.kBack.value).whileTrue(new ElevatorDown(m_elevatorSubsytem));
		// // EC force elevator up
		// new JoystickButton(Constants.emergencyController, XboxController.Button.kStart.value).whileTrue(new ElevatorDown(m_elevatorSubsytem, .4));
		
		// new JoystickButton(Constants.emergencyController, 5).onTrue(new ArmToWPI(ArmLocation.HIGH_CORAL, m_armSubsystem));
		new JoystickButton(Constants.emergencyController, XboxController.Button.kX.value)
				.whileTrue(Autos.driveSysIdRoutine(m_driveSubsystem, RoutineType.QUASISTATIC, Direction.kForward));

		new JoystickButton(Constants.emergencyController, XboxController.Button.kY.value)
				.whileTrue(Autos.driveSysIdRoutine(m_driveSubsystem, RoutineType.DYNAMIC, Direction.kForward));

		new JoystickButton(Constants.emergencyController, XboxController.Button.kRightBumper.value)
				.whileTrue(Autos.driveSysIdRoutine(m_driveSubsystem, RoutineType.QUASISTATIC, Direction.kReverse));

		new JoystickButton(Constants.emergencyController, XboxController.Button.kLeftBumper.value)
				.whileTrue(Autos.driveSysIdRoutine(m_driveSubsystem, RoutineType.DYNAMIC, Direction.kReverse));

		new JoystickButton(Constants.emergencyController, XboxController.Button.kA.value)
				.whileTrue(m_driveSubsystem.run(() -> {
						double voltage = SmartDashboard.getNumber("volts", 0);
						m_driveSubsystem.setChassisVoltage(voltage);
					}));

		new JoystickButton(Constants.emergencyController, XboxController.Button.kB.value)
					.whileTrue(m_driveSubsystem.runEnd(() -> {
						m_driveSubsystem.blFeedForward.setKv(SmartDashboard.getNumber("bl - kV", Constants.BL_kV));
						m_driveSubsystem.brFeedForward.setKv(SmartDashboard.getNumber("br - kV", Constants.BR_kV));
						m_driveSubsystem.flFeedForward.setKv(SmartDashboard.getNumber("fl - kV", Constants.FL_kV));
						m_driveSubsystem.frFeedForward.setKv(SmartDashboard.getNumber("fr - kV", Constants.FR_kV));

						m_driveSubsystem.setChassisFeedForward(1.5);

						// System.out.println("bl - kV: " + m_driveSubsystem.blFeedForward.getKv());
						// System.out.println("bl - rpm: " + m_driveSubsystem.blEncoder.getVelocity());
						// System.out.println("bl - m/s: " + m_driveSubsystem.blEncoder.getVelocity() / 60 * Constants.WHEEL_DIAMETER.in(Units.Meters) * Math.PI);
						// System.out.println();
						// System.out.println("br - kV: " + m_driveSubsystem.brFeedForward.getKv());
						// System.out.println("br - rpm: " + m_driveSubsystem.brEncoder.getVelocity());
						// System.out.println("br - m/s: " + m_driveSubsystem.brEncoder.getVelocity() / 60 * Constants.WHEEL_DIAMETER.in(Units.Meters) * Math.PI);
						// System.out.println();
						// System.out.println("fl - kV: " + m_driveSubsystem.flFeedForward.getKv());
						// System.out.println("fl - rpm: " + m_driveSubsystem.flEncoder.getVelocity());
						// System.out.println("fl - m/s: " + m_driveSubsystem.flEncoder.getVelocity() / 60 * Constants.WHEEL_DIAMETER.in(Units.Meters) * Math.PI);
						// System.out.println();
						// System.out.println("fr - kV: " + m_driveSubsystem.frFeedForward.getKv());
						// System.out.println("fr - rpm: " + m_driveSubsystem.frEncoder.getVelocity());
						// System.out.println("fr - m/s: " + m_driveSubsystem.frEncoder.getVelocity() / 60 * Constants.WHEEL_DIAMETER.in(Units.Meters) * Math.PI);
						// System.out.println();
						// System.out.println();
					}, () -> {m_driveSubsystem.setChassisVoltage(0);}));

		new JoystickButton(Constants.emergencyController, XboxController.Button.kA.value).whileTrue(m_driveSubsystem.runEnd(
			() -> {
				double speed = SmartDashboard.getNumber("chassisSpeed", 0);
				double theta = SmartDashboard.getNumber("theta", 0);

				double thetaRads = theta * Math.PI / 180;
				double strafeSpeedAccount = 0.82;
				m_driveSubsystem.driveRobotSpeed(new ChassisSpeeds(Units.MetersPerSecond.of(speed * Math.cos(thetaRads)), Units.MetersPerSecond.of(speed * Math.sin(thetaRads) / strafeSpeedAccount), Units.DegreesPerSecond.of(0)));
			}, () -> {
				m_driveSubsystem.setChassisVoltage(0);
			}
		));
		// new JoystickButton(Constants.primaryStick, 12).whileTrue(new
		// SequentialCommandGroup(
		// new ArmToWPI(Units.Degrees.of(120), m_armSubsystem),
		// new ParallelCommandGroup(
		// m_rollerSubsystem.startEnd(() -> {m_rollerSubsystem.roller.set(.25);}, () ->
		// {m_rollerSubsystem.roller.set(0);m_rollerSubsystem.rollerEncoder.setPosition(0);}),
		// m_driveSubsystem.startEnd(() -> {m_driveSubsystem.mechDrive(.25, 0, 0);}, ()
		// -> {m_driveSubsystem.mechDrive(0, 0, 0);}).repeatedly()
		// )
		// // new ParallelCommandGroup(
		// // m_rollerSubsystem.startEnd(() -> (m_rollerSubsystem.roller.set(-0.5)),
		// null)
		// // )

		// ));

		SmartDashboard.putData("Out", new ArmToWPI(ArmLocation.DURING_ELEVATOR_MOVEMENT, m_armSubsystem));
		SmartDashboard.putData("Int: ", new ArmToWPI(ArmLocation.INTAKE, m_armSubsystem));
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		// An example command will be run in autonomous
		// Command auton = m_chooser.getSelected();
		// if (auton == null) auton = Autos.moveForward(5, m_driveSubsystem);
		return new SequentialCommandGroup(
			Autos.driveSysIdRoutine(m_driveSubsystem, RoutineType.QUASISTATIC, Direction.kForward)
			// new Wait(5),
			// Autos.driveSysIdRoutine(m_driveSubsystem, RoutineType.DYNAMIC, Direction.kForward),
			// new Wait(5),
			// Autos.driveSysIdRoutine(m_driveSubsystem, RoutineType.QUASISTATIC, Direction.kReverse),
			// new Wait(5),
			// Autos.driveSysIdRoutine(m_driveSubsystem, RoutineType.DYNAMIC, Direction.kReverse)
		);
	}
}
