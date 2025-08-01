// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.arm.Arm;
import frc.robot.arm.ArmHw;
import frc.robot.intake.Intake;
import frc.robot.intake.IntakeHw;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.ElevatorHw;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  Elevator elevator;
  Arm arm;
  Intake intake;

  SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    elevator = new ElevatorHw();
    arm = new ArmHw();
    intake = new IntakeHw();
    
    elevator.setDefaultCommand(elevator.holdElevator());
    arm.setDefaultCommand(arm.holdArm());

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);


    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {


  }



}
