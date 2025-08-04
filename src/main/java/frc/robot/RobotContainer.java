// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Set;

import com.fasterxml.jackson.databind.util.Named;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.arm.Arm;
import frc.robot.arm.ArmHw;
import frc.robot.controllers.DriverControls;
import frc.robot.intake.Intake;
import frc.robot.intake.IntakeHw;
import frc.robot.mecanumdrive.DriveSubsystem;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.ScoringPositions;
import frc.robot.vision.Vision;
import frc.robot.vision.Vision.Poles;
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
    
    Vision vision;
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

        vision = new Vision(driveSubsystem);
        driveSubsystem.setVisionSubsystem(vision);

        /* The bot will align with the center of the reef (to take off algae) instead of to the sides (to place coral)
         * whenever it is in zone D */
        vision.addCoralModeSupplier(() -> getZone(elevator.getPosition(), arm.getAngle()) != Zones.ZoneD);

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        NamedCommands.registerCommand("FineDriveA", driveSubsystem.alignToPoleDeferred(Poles.PoleA));
        NamedCommands.registerCommand("FineDriveB", driveSubsystem.alignToPoleDeferred(Poles.PoleB));
        NamedCommands.registerCommand("FineDriveC", driveSubsystem.alignToPoleDeferred(Poles.PoleC));
        NamedCommands.registerCommand("FineDriveD", driveSubsystem.alignToPoleDeferred(Poles.PoleD));
        NamedCommands.registerCommand("FineDriveE", driveSubsystem.alignToPoleDeferred(Poles.PoleE));
        NamedCommands.registerCommand("FineDriveF", driveSubsystem.alignToPoleDeferred(Poles.PoleF));
        NamedCommands.registerCommand("FineDriveG", driveSubsystem.alignToPoleDeferred(Poles.PoleG));
        NamedCommands.registerCommand("FineDriveH", driveSubsystem.alignToPoleDeferred(Poles.PoleH));
        NamedCommands.registerCommand("FineDriveI", driveSubsystem.alignToPoleDeferred(Poles.PoleI));
        NamedCommands.registerCommand("FineDriveJ", driveSubsystem.alignToPoleDeferred(Poles.PoleJ));
        NamedCommands.registerCommand("FineDriveK", driveSubsystem.alignToPoleDeferred(Poles.PoleK));
        NamedCommands.registerCommand("FineDriveL", driveSubsystem.alignToPoleDeferred(Poles.PoleL));
        NamedCommands.registerCommand("ElevatorL1Coral", setScoringPosition(ScoringPositions.L1Coral));
        NamedCommands.registerCommand("ElevatorL2Coral", setScoringPosition(ScoringPositions.L2Coral));
        NamedCommands.registerCommand("ElevatorL3Coral", setScoringPosition(ScoringPositions.L3Coral));
        NamedCommands.registerCommand("ElevatorL4Coral", setScoringPosition(ScoringPositions.L4Coral));
        NamedCommands.registerCommand("ElevatorL2Algae", setScoringPosition(ScoringPositions.L2Algae));
        NamedCommands.registerCommand("ElevatorL3Algae", setScoringPosition(ScoringPositions.L3Algae));
        NamedCommands.registerCommand("ElevatorLoading", setScoringPosition(ScoringPositions.LoadingPosition));
        NamedCommands.registerCommand("HomeCoral", homeCoral());

        // Configure the trigger bindings
        configureBindings();
    }

    private void configureBindings() {
        DriverControls driver = new DriverControls();

        driveSubsystem.setDefaultCommand(
                driveSubsystem.driveCommandRobotCentric(driver::getDriveX, driver::getDriveY, driver::getTurn));
        // elevator.setDefaultCommand(elevator.holdElevator());
        // arm.setDefaultCommand(arm.holdArm());

        // driver.driveToPole().whileTrue(driveSubsystem.alignToClosestPole());
    }

    enum Zones {
        ZoneA, ZoneB, ZoneC, ZoneD, ZoneE, ZoneF
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
