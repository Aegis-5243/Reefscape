// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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

    // elevator.setDefaultCommand(elevator.holdElevator());
    // arm.setDefaultCommand(arm.holdArm());

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
    // Zone E - The positions to be scored from; the arm is potentially
    // ........ intersecting the elevator check intersections on cad
    // ........ before making edits
    // Zone F - The arm definitely intersects the elevator
    if (angle >= 100) {
      return Zones.ZoneD;
    } else if (angle >= 58) {
      return Zones.ZoneC;
    } else if (height < 2 && angle <= 25) {
      return Zones.ZoneA;
    } else if (height < 7 && angle <= 25) {
      return Zones.ZoneB;
    } else if (angle >= 40) {
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

  private Command setScoringPositionDeferred(ScoringPositions position) { // TODO: test
    // state machine like logic from
    // https://github.com/FRC2832/Robot2832-2025Njord/blob/a11e334a0eab59214d62ff34fd51ab5178f034c5/src/main/java/frc/robot/RobotContainer.java#L405
    Zones currZone = getZone(elevator.getPosition(), arm.getAngle());
    Zones destZone = getZone(elevator.getSetPosition(position), arm.getSetPosition(position));

    Command result;

    if (currZone == Zones.ZoneE || currZone == Zones.ZoneF) { // E or F to C initially since E and F cannot
      result = arm.setAngleCmd(65)
          .until(() -> arm.getAngle() > 60)
          .andThen(setScoringPosition(position));
    } else if (currZone == destZone) { // Already in the correct zone, no potential collisions
      result = new ParallelCommandGroup(arm.setAngleCmd(position),
          elevator.setPositionCmd(position));
    } else if (currZone == Zones.ZoneD) { // D to C then repeat
      result = arm.setAngleCmd(60)
          .until(() -> arm.getAngle() < 65)
          .andThen(setScoringPosition(position));
    } else if (currZone == Zones.ZoneB) { // B to A or C
      if (destZone == Zones.ZoneA) { // B to A
        result = new ParallelCommandGroup(
            arm.setAngleCmd(position),
            elevator.setPositionCmd(position));
      } else { // B to A then repeat
        result = elevator.setPositionCmd(0)
            .andThen(setScoringPosition(position));
      }
    } else if (currZone == Zones.ZoneA) { // A to B or C
      if (destZone == Zones.ZoneB) { // A to B
        result = new ParallelCommandGroup(
            arm.setAngleCmd(position),
            elevator.setPositionCmd(position));
      } else { // A to C then repeat
        result = arm.setAngleCmd(65)
            .until(() -> arm.getAngle() > 60)
            .andThen(setScoringPosition(position));
      }
    } else if (currZone == Zones.ZoneC) { // C to A, B, D, or E
      if (destZone == Zones.ZoneB) { // C to B
        result = elevator.setPositionCmd(0)
            .andThen(arm.setAngleCmd(position))
            .andThen(elevator.setPositionCmd(position));
      } else if (destZone == Zones.ZoneA) {
        result = elevator.setPositionCmd(position)
            .andThen(arm.setAngleCmd(position));
      } else if (destZone == Zones.ZoneD) {
        result = new ParallelCommandGroup(
            arm.setAngleCmd(position),
            elevator.setPositionCmd(position));
      } else if (destZone == Zones.ZoneE) {
        result = elevator.setPositionCmd(position)
            .andThen(arm.setAngleCmd(position));
      } else {
        result = Commands.none();
      }
    } else {
      result = Commands.none();
    }

    return result;
  }

  private Command homeCoral() {
    return new SequentialCommandGroup(
        intake.setVelocityCmd(12)
            .until(() -> intake.detectingCoral()),
        intake.setVelocityCmd(-1)
            .until(() -> !intake.detectingCoral()),
        intake.setPositionCmd(() -> intake.getPosition() + 2.8));
  }

}
