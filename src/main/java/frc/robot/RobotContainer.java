// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.arm.Arm;
import frc.robot.arm.ArmHw;
import frc.robot.intake.Intake;
import frc.robot.intake.IntakeHw;
import frc.robot.mechanumdrive.DriveSubsystem;
import frc.robot.utils.ScoringPositions;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.ElevatorHw;

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
  DriveSubsystem driveSubsystem = new DriveSubsystem();

  Elevator elevator;
  Arm arm;
  Intake intake;

  SendableChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
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

  enum Zones {
    ZoneA,
    ZoneB,
    ZoneC,
    ZoneD,
    ZoneE,
    ZoneF
  }

  private Zones getZone(double height, double angle) {
    // Zone A - Intake area with elevator down
    // Zone B - Intake area with elevator up, need to go down to A
    // Zone C - Coral scoring
    // Zone D - Algae removal
    // Zone E - Arm is above elevator, go to C
    // Zone F - Arm potentially intersecting elevator, needed for some 
    // ........ scoring so check intersections on cad before makingedits
    if (angle >= 100) {
      return Zones.ZoneD;
    } else if (angle >= 55) {
      return Zones.ZoneC;
    } else if (height < 2 && angle <= 25) {
      return Zones.ZoneA;
    } else if (height < 7 && angle <= 25) {
      return Zones.ZoneB;
    } else if (height > 45) {
      return Zones.ZoneE;
    } else {
      return Zones.ZoneF;
    }
  }

  /*
   * Potential implementations
   * - better approach to detecting coral
   * . - currently using a single sensor within the arm
   * with a refresh period of 24ms
   * 
   * - avoid getting close to the coral recieving zone
   * when not in intake position (Zone A)
   * 
   */

  private Command setScoringPosition(ScoringPositions position) {
    return new DeferredCommand(() -> setScoringPositionDeferred(position), Set.of(elevator, arm));
  }

  private Command setScoringPositionDeferred(ScoringPositions position) {
    Zones currZone = getZone(elevator.getPosition(), arm.getAngle());
    Zones destZone = getZone(elevator.getSetPosition(position), arm.getSetPosition(position));

    Command result;

    if (currZone == Zones.ZoneF) {
      result = arm.setAngleCmd(65)
          .until(() -> arm.getAngle() > 60)
          .andThen(setScoringPosition(position));
    } else if (currZone == destZone) {
      result = new ParallelCommandGroup(
          elevator.setPositionCmd(position),
          arm.setAngleCmd(position));
    } else if (currZone == Zones.ZoneB) { // Go to A to avoid elevator
      result = elevator.setPositionCmd(0).andThen(setScoringPosition(position));
    } else if (destZone == Zones.ZoneC) {
      result = arm.setAngleCmd(position)
          .andThen(elevator.setPositionCmd(position));
    } else if (destZone == Zones.ZoneF) {
      result = arm.setAngleCmd(65)
          .until(() -> arm.getAngle() > 60)
          .andThen(elevator.setPositionCmd(position))
          .andThen(arm.setAngleCmd(position));
    } else if ((currZone == Zones.ZoneA || currZone == Zones.ZoneE) && destZone == Zones.ZoneC) { // E or A to C
      result = arm.setAngleCmd(65)
          .until(() -> arm.getAngle() > 60)
          .andThen(setScoringPosition(position));
    } else if (currZone == Zones.ZoneC && destZone == Zones.ZoneD) { // C to D
      result = arm.setAngleCmd(100)
          .until(() -> arm.getAngle() > 90)
          .andThen(setScoringPosition(position));
    } else {
      result = null;
    }

    return result;
  }
}
