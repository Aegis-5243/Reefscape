// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.PPLibTelemetry;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.controllers.ProConControls;
import frc.robot.controllers.XBoxControls;

/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the
 * package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer robotContainer;
    
        static boolean teleopFlag = false;
        static boolean testFlag = false;

        public static boolean robotIsTeleop() {
        return teleopFlag;
        }

        public static boolean robotIsTest() {
            return testFlag;
        }
  
    
        /**
         * This function is run when the robot is first started up and should be used
         * for any
         * initialization code.
         */
        public Robot() {
    
            // add webserver to allow download of the dashboard in Elastic
            WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
    
            // turn off hot reload at the competition. (from 2832)
            if (DriverStation.isFMSAttached()) {
                PPLibTelemetry.enableCompetitionMode();
            }
    
            SmartDashboard.putData(CommandScheduler.getInstance());

            robotContainer = new RobotContainer();
        }
    
        /**
         * This function is called every 20 ms, no matter the mode. Use this for items
         * like diagnostics
         * that you want ran during disabled, autonomous, teleoperated and test.
         *
         * <p>
         * This runs after the mode specific periodic functions, but before LiveWindow
         * and
         * SmartDashboard integrated updating.
         */
        @Override
        public void robotPeriodic() {
            // Runs the Scheduler. This is responsible for polling buttons, adding
            // newly-scheduled
            // commands, running already-scheduled commands, removing finished or
            // interrupted commands,
            // and running subsystem periodic() methods. This must be called from the
            // robot's periodic
            // block in order for anything in the Command-based framework to work.
            CommandScheduler.getInstance().run();
        }
    
        /** This function is called once each time the robot enters Disabled mode. */
        @Override
        public void disabledInit() {
            teleopFlag = false;
        }
    
        @Override
        public void disabledPeriodic() {
        }
    
        /**
         * This autonomous runs the autonomous command selected by your
         * {@link RobotContainer} class.
         */
        @Override
        public void autonomousInit() {
            teleopFlag = false;
            m_autonomousCommand = robotContainer.getAutonomousCommand();

            // schedule the autonomous command (example)
            if (m_autonomousCommand != null) {
                m_autonomousCommand.schedule();
            }
        }
    
        /** This function is called periodically during autonomous. */
        @Override
        public void autonomousPeriodic() {
        }
    
        @Override
        public void teleopInit() {
            teleopFlag = true;
            testFlag = false;
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        } else {
            CommandScheduler.getInstance().cancelAll();
        }

        robotContainer.reset();
    }

    Debouncer xBoxFullStopDebouncer = new Debouncer(0.25, DebounceType.kRising);

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        if (robotContainer.driver instanceof ProConControls) {
            ProConControls driv = (ProConControls) robotContainer.driver;
            if (!driv.controller.isConnected() || xBoxFullStopDebouncer.calculate(driv.getStopTeleop())) {
                // Apparently teleOp can't be cancelled so we'll just call the reset command
                robotContainer.reset();
            }
        } else if (robotContainer.driver instanceof XBoxControls) {
            XBoxControls driv = (XBoxControls) robotContainer.driver;
            if (!driv.controller.isConnected() || xBoxFullStopDebouncer.calculate(driv.getStopTeleop())) {
                // Apparently teleOp can't be cancelled so we'll just call the reset command
                robotContainer.reset();
            }
        }
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
        teleopFlag = true;
        testFlag = true;
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
        if (robotContainer.driver instanceof ProConControls) {
            ProConControls driv = (ProConControls) robotContainer.driver;
            if (!driv.controller.isConnected() || xBoxFullStopDebouncer.calculate(driv.getStopTeleop())) {
                // Apparently teleOp can't be cancelled so we'll just call the reset command
                robotContainer.reset();
            }
        } else if (robotContainer.driver instanceof XBoxControls) {
            XBoxControls driv = (XBoxControls) robotContainer.driver;
            if (!driv.controller.isConnected() || xBoxFullStopDebouncer.calculate(driv.getStopTeleop())) {
                // Apparently teleOp can't be cancelled so we'll just call the reset command
                robotContainer.reset();
            }
        }
    }

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {
    }

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {
    }
}
