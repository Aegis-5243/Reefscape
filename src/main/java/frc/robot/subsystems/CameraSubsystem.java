// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CameraSubsystem extends SubsystemBase {
    /** Creates a new ExampleSubsystem. */
    private static CameraSubsystem instance;

    public CameraSubsystem() {
        instance = this;
        CameraServer.startAutomaticCapture(0);
        CameraServer.startAutomaticCapture(new HttpCamera(Constants.FRONT_LIMELIGHT, "http://10.52.43.201:5800"));
        CameraServer.startAutomaticCapture(new HttpCamera(Constants.BACK_LIMELIGHT, "http://10.52.43.200:5800/"));
    }

    public static CameraSubsystem getInstance() {
        return instance;
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
        // CameraServer.startAutomaticCapture(new HttpCamera(Constants.FRONT_LIMELIGHT, "http://10.52.43.201:5800"));
        // CameraServer.startAutomaticCapture(new HttpCamera(Constants.BACK_LIMELIGHT, "http://10.52.43.200:5800/"));
        // This method will be called once per scheduler run

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
