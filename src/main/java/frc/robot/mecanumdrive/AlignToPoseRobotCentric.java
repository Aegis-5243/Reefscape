package frc.robot.mecanumdrive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.util.UtilFunctions;

/* https://github.com/FRC2832/Robot2832-2025Njord/blob/main/src/main/java/frc/robot/swervedrive/AlignToPose.java */
public class AlignToPoseRobotCentric extends Command {
    private PIDController xController, yController, rotController;
    private DriveSubsystem drive;
    private ChassisSpeeds noSpeeds;
    private double lastHeading;
    private Pose2d target;
    private int counts;
    private int maxCounts;
    private double maxErrorLimit;

    DoubleSubscriber kP = UtilFunctions.getSettingSub("Align/kP", Constants.DRIVE_kP);
    DoubleSubscriber kI = UtilFunctions.getSettingSub("Align/kI", Constants.DRIVE_kI);
    DoubleSubscriber kD = UtilFunctions.getSettingSub("Align/kD", Constants.DRIVE_kD);

    DoubleSubscriber rkP = UtilFunctions.getSettingSub("Align/rkP", Constants.DRIVE_ROTATE_kP);
    DoubleSubscriber rkI = UtilFunctions.getSettingSub("Align/rkI", Constants.DRIVE_ROTATE_kI); // TODO: restore default preference
    DoubleSubscriber rkD = UtilFunctions.getSettingSub("Align/rkD", Constants.DRIVE_ROTATE_kD);

    public AlignToPoseRobotCentric(DriveSubsystem swerve, Pose2d pose) {
        this(swerve, pose, 6, 1.2);
    }

    public AlignToPoseRobotCentric(DriveSubsystem swerve, Pose2d pose, int endCounts) {
        this(swerve, pose, endCounts, 1.2);
    }

    public AlignToPoseRobotCentric(DriveSubsystem swerve, Pose2d pose, int endCounts, double errorDist) {
        this.drive = swerve;
        this.target = pose;
        this.maxCounts = endCounts;
        this.maxErrorLimit = errorDist;
        noSpeeds = new ChassisSpeeds();

        xController = new PIDController(kP.getAsDouble(), kI.getAsDouble(), kD.getAsDouble()); // Vertical movement
        yController = new PIDController(kP.getAsDouble(), kI.getAsDouble(), kD.getAsDouble()); // Horitontal movement
        rotController = new PIDController(rkP.getAsDouble(), rkI.getAsDouble(), rkD.getAsDouble()); // Rotation
        rotController.enableContinuousInput(-180, 180);

        rotController.setSetpoint(pose.getRotation().getDegrees());
        rotController.setTolerance(4);
        lastHeading = pose.getRotation().getDegrees();

        // xController.setSetpoint(pose.getX());
        xController.setSetpoint(0);
        xController.setTolerance(Units.inchesToMeters(0.9));

        // yController.setSetpoint(pose.getY());
        yController.setSetpoint(0);
        yController.setTolerance(Units.inchesToMeters(0.9));
        
        this.setName("Align to pose (RC) " + pose);
    }

    @Override
    public void initialize() {
        counts = 0;
    }

    @Override
    public void execute() {
        Pose2d pose = drive.getPose();
        var xError = Units.metersToInches(pose.getX() - target.getX());
        var yError = Units.metersToInches(pose.getY() - target.getY());
        var rotError = pose.getRotation().minus(target.getRotation()).getDegrees();

        // handle 360 circle problem
        double curHeading = pose.getRotation().getDegrees();
        // double centeredHeading =
        // MathUtil.inputModulus(curHeading, lastHeading - 180, lastHeading + 180);
        double rotValue = rotController.calculate(curHeading);
        lastHeading = curHeading;

        Pose2d robotCentricError = target.relativeTo(pose);

        // TODO: test VERY CAREFULLY
        // rollback to "after WW planning meeting" to undo changes here

        double xSpeed = xController.calculate(-robotCentricError.getX());
        double ySpeed = yController.calculate(-robotCentricError.getY());

        SmartDashboard.putNumber("Align/xError", Units.metersToInches(robotCentricError.getX()));
        SmartDashboard.putNumber("Align/yError", Units.metersToInches(robotCentricError.getY()));
        SmartDashboard.putNumber("Align/rotError", rotError);

        // send the drive command
        ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, rotValue);
        // drive.driveFieldOriented(speeds);
        drive.drive(speeds);

        // calculate if we are at goal
        // error was 0.8" X, 1.1" Y, or 1.36" total
        double distError = Math.sqrt((xError * xError) + (yError * yError));
        if (Math.abs(rotError) < 2 && distError < maxErrorLimit) {
            counts++;
        } else {
            // either decrement the count or make it zero
            counts = Math.max(counts--, 0);
        }
    }

    @Override
    public boolean isFinished() {
        return counts >= maxCounts;
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(noSpeeds);
    }
}