// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CameraSubsystem extends SubsystemBase {
    
    private static CameraSubsystem instance;
    public UsbCamera lifecam;
    public HttpCamera limelight1;
    public HttpCamera limelight2;

    public CvSink lifecamCvSink;
    public CvSink limelight1CvSink;
    public CvSink limelight2CvSink;

    public MjpegServer lifecamServer;
    public MjpegServer limelight1Server;
    public MjpegServer limelight2Server;

    public CvSource outputStream;

    public Mat frame;

    public Point pt1;
    public Point pt2;
    public Point pt3;
    public Point pt4;
    public Scalar color;

    /**
     * Creates a new CameraSubsystem
     */
    public CameraSubsystem() {
        instance = this;

        lifecam = new UsbCamera("USB Camera 0", 0);
        limelight1 = new HttpCamera(Constants.FRONT_LIMELIGHT, "http://10.52.43.201:5800");
        limelight2 = new HttpCamera(Constants.BACK_LIMELIGHT, "http://10.52.43.200:5800/");

        lifecamServer = CameraServer.startAutomaticCapture(lifecam);
        limelight1Server = CameraServer.startAutomaticCapture(limelight2);
        limelight2Server = CameraServer.startAutomaticCapture(limelight2);
        
        lifecamCvSink = new CvSink("lifecam", PixelFormat.kMJPEG);
        lifecamCvSink.setSource(lifecam);
        outputStream = CameraServer.putVideo("lifecam-crosshair", 320, 240);
        
        new Thread(() -> {
            frame = new Mat();
            int x = 120;
            int y = 120;
            pt1 = new Point(x -120 + 120, y - 120 + 120);
            pt2 = new Point(x -120 + 200, y - 120 + 120);
            pt3 = new Point(x -120 + 160, y - 120 + 80);
            pt4 = new Point(x -120 + 160, y - 120 + 160);
            color = new Scalar(255, 0, 0);

            
            while(!Thread.interrupted()) {
                if (lifecamCvSink.grabFrame(frame) == 0) {
                continue;
                }
                Imgproc.line(frame, pt1, pt2, color, 3);
                Imgproc.line(frame, pt3, pt4, color, 3);
                outputStream.putFrame(frame);
            }
        }).start();
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
        // This method will be called once per scheduler run
        
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
