package frc.robot.mecanumdrive;

import java.util.Optional;
import java.util.Set;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AlternateEncoderConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.UtilFunctions;
import frc.robot.vision.Vision;
import frc.robot.vision.Vision.Poles;

public class DriveSubsystem extends SubsystemBase {

    public SparkMax fl;
    public SparkMax fr;
    public SparkMax bl;
    public SparkMax br;

    public RelativeEncoder flEncoder;
    public RelativeEncoder frEncoder;
    public RelativeEncoder blEncoder;
    public RelativeEncoder brEncoder;

    public SimpleMotorFeedforward flFeedForward;
    public SimpleMotorFeedforward frFeedForward;
    public SimpleMotorFeedforward blFeedForward;
    public SimpleMotorFeedforward brFeedForward;

    public MecanumDrive mecanum;

    public AHRS gyro;

    public MecanumDriveKinematics kinematics;

    public MecanumDrivePoseEstimator poseEstimator;

    public Field2d field;

    public FieldObject2d robotObject;

    private DoubleSubscriber strafeSpeedScale;
    private DoubleSubscriber turnSpeedScale;

    private Vision vision;

    public DriveSubsystem() {

        strafeSpeedScale = UtilFunctions.getSettingSub("mecanum/strafeSpeedScale", 1.22);
        turnSpeedScale = UtilFunctions.getSettingSub("mecanum/turnSpeedScale", 1.08);

        fl = new SparkMax(Constants.FL, MotorType.kBrushless);
        fr = new SparkMax(Constants.FR, MotorType.kBrushless);
        bl = new SparkMax(Constants.BL, MotorType.kBrushless);
        br = new SparkMax(Constants.BR, MotorType.kBrushless);

        SparkBaseConfig motorConfig = new SparkMaxConfig()
                .apply(new AlternateEncoderConfig()
                        .positionConversionFactor(
                                Constants.MECANUM_ALTERNATE_POSITION_CONVERSION_FACTOR)
                        .velocityConversionFactor(
                                Constants.MECANUM_ALTERNATE_VELOCITY_CONVERSION_FACTOR)
                        .setSparkMaxDataPortConfig())
                .apply(
                        new EncoderConfig()
                                .positionConversionFactor(
                                        Constants.MECANUM_POSITION_CONVERSION_FACTOR)
                                .velocityConversionFactor(
                                        Constants.MECANUM_VELOCITY_CONVERSION_FACTOR))
                .idleMode(IdleMode.kCoast)
                .closedLoopRampRate(.5)
                .openLoopRampRate(.5);

        // Match inversions on motor and alternate encoder and apply global config
        fl.configure(new SparkMaxConfig()
                .apply(new AlternateEncoderConfig().inverted(false))
                .inverted(false)
                .apply(motorConfig), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        fr.configure(new SparkMaxConfig()
                .apply(new AlternateEncoderConfig().inverted(true))
                .inverted(true)
                .apply(motorConfig), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        bl.configure(new SparkMaxConfig()
                .apply(new AlternateEncoderConfig().inverted(false))
                .inverted(false)
                .apply(motorConfig), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        br.configure(new SparkMaxConfig()
                .apply(new AlternateEncoderConfig().inverted(true))
                .inverted(true)
                .apply(motorConfig), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Able to substitute since conversion factors match gearbox ratio of 12.75
        flEncoder = fl.getEncoder();
        frEncoder = fr.getAlternateEncoder();
        blEncoder = bl.getAlternateEncoder();
        brEncoder = br.getAlternateEncoder();

        flFeedForward = new SimpleMotorFeedforward(Constants.FL_kS, Constants.FL_kV, Constants.FL_kA);
        frFeedForward = new SimpleMotorFeedforward(Constants.FR_kS, Constants.FR_kV, Constants.FR_kA);
        blFeedForward = new SimpleMotorFeedforward(Constants.BL_kS, Constants.BL_kV, Constants.BL_kA);
        brFeedForward = new SimpleMotorFeedforward(Constants.BR_kS, Constants.BR_kV, Constants.BR_kA);

        gyro = new AHRS(NavXComType.kUSB1);

        gyro.reset();

        mecanum = new MecanumDrive(fl, bl, fr, br);

        kinematics = new MecanumDriveKinematics(new Translation2d(0.259, 0.283),
                new Translation2d(0.259, -0.283),
                new Translation2d(-0.259, 0.283), new Translation2d(-0.259, -0.283));

        poseEstimator = new MecanumDrivePoseEstimator(kinematics, getHeading(),
                new MecanumDriveWheelPositions(),
                Pose2d.kZero);

        setUpPathPlanner();
        field = new Field2d();
        robotObject = field.getRobotObject();

        Shuffleboard.getTab("Teleoperated").add(field)
                .withPosition(0, 0)
                .withSize(6, 3);

        ShuffleboardTab tab = Shuffleboard.getTab("Drive");
        tab.addFloatArray("Encoder Velocities", () -> {
            return new float[] {
                    (float) fl.getEncoder().getVelocity(),
                    (float) fr.getEncoder().getVelocity(),
                    (float) bl.getEncoder().getVelocity(),
                    (float) br.getEncoder().getVelocity()
            };
        })
                .withPosition(0, 0)
                .withSize(2, 1);
        tab.addFloatArray("Alternate Encoder Velocities", () -> {
            return new float[] {
                    (float) fl.getAlternateEncoder().getVelocity(),
                    (float) fr.getAlternateEncoder().getVelocity(),
                    (float) bl.getAlternateEncoder().getVelocity(),
                    (float) br.getAlternateEncoder().getVelocity()
            };
        })
                .withPosition(0, 1)
                .withSize(2, 1);
        tab.addFloatArray("Encoder Positions", () -> {
            return new float[] {
                    (float) fl.getEncoder().getPosition(),
                    (float) fr.getEncoder().getPosition(),
                    (float) bl.getEncoder().getPosition(),
                    (float) br.getEncoder().getPosition()
            };
        })
                .withPosition(0, 2)
                .withSize(2, 1);
        tab.addFloatArray("Alternate Encoder Positions", () -> {
            return new float[] {
                    (float) fl.getAlternateEncoder().getPosition(),
                    (float) fr.getAlternateEncoder().getPosition(),
                    (float) bl.getAlternateEncoder().getPosition(),
                    (float) br.getAlternateEncoder().getPosition()
            };
        })
                .withPosition(0, 3)
                .withSize(2, 1);
        tab.addFloatArray("Motor Voltages", () -> {
            return new float[] {
                    (float) (fl.getBusVoltage() * fl.getAppliedOutput()),
                    (float) (fr.getBusVoltage() * fr.getAppliedOutput()),
                    (float) (bl.getBusVoltage() * bl.getAppliedOutput()),
                    (float) (br.getBusVoltage() * br.getAppliedOutput())
            };
        });
        tab.addDouble("xSpeed", () -> getChassisSpeeds().vxMetersPerSecond)
                .withPosition(3, 0)
                .withSize(1, 1);
        tab.addDouble("ySpeed", () -> getChassisSpeeds().vyMetersPerSecond)
                .withPosition(4, 0)
                .withSize(1, 1);
        tab.addDouble("zRotation", () -> getChassisSpeeds().omegaRadiansPerSecond)
                .withPosition(5, 0)
                .withSize(1, 1);
        tab.addDouble("xPosition", () -> getPose().getX())
                .withPosition(3, 1)
                .withSize(1, 1);
        tab.addDouble("yPosition", () -> getPose().getY())
                .withPosition(4, 1)
                .withSize(1, 1);
        tab.addDouble("Heading", () -> getHeading().getDegrees())
                .withPosition(5, 1)
                .withSize(1, 1);

    }

    public void setVisionSubsystem(Vision visionSubsystem) {
        vision = visionSubsystem;
    }

    @Override
    public void periodic() {
        poseEstimator.update(getHeading(), new MecanumDriveWheelPositions(
                fl.getEncoder().getPosition(),
                fr.getEncoder().getPosition(),
                bl.getEncoder().getPosition(),
                br.getEncoder().getPosition()));

        updateFieldPoseEstimate();
    }

    public void setUpPathPlanner() {
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // TODO: handle exception
            config = null;
            e.printStackTrace();
        }

        AutoBuilder.configure(
                this::getPose,
                this::resetPose,
                this::getChassisSpeeds,
                this::drive,
                new PPHolonomicDriveController(
                        new PIDConstants(1, 0, 0),
                        new PIDConstants(1, 0, 0)),
                config,
                () -> {
                    Optional<Alliance> alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this);
    }

    public boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
    }

    public void driveCartesian(double xSpeed, double ySpeed, double zRotation) {
        mecanum.driveCartesian(xSpeed, ySpeed, zRotation);
    }

    public Command driveCommandRobotCentric(
            DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX) {
        return run(() -> driveCartesian(
                translationX.getAsDouble(),
                translationY.getAsDouble(),
                angularRotationX.getAsDouble()));

    }

    public Rotation2d getHeading() {
        return gyro.getRotation2d();
    }

    public void resetGyro() {
        gyro.zeroYaw();
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void resetPose(Pose2d pose) {
        poseEstimator.resetPose(pose);
    }

    public Command resetPoseCommand(Pose2d pose) {
        return runOnce(() -> resetPose(pose));
    }

    public void updateFieldPoseEstimate() {
        field.setRobotPose(getPose());
    }

    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(
                new MecanumDriveWheelSpeeds(
                        flEncoder.getVelocity(),
                        frEncoder.getVelocity(),
                        blEncoder.getVelocity(),
                        brEncoder.getVelocity()));
    }

    public void drive(ChassisSpeeds speeds) {
        double absSpeed = Math.sqrt(speeds.vxMetersPerSecond * speeds.vxMetersPerSecond
                + speeds.vyMetersPerSecond * speeds.vyMetersPerSecond);

        if (absSpeed > Constants.DRIVE_MAX_SPEED * Constants.DRIVE_MAX_SPEED) {
            speeds.vxMetersPerSecond *= Constants.DRIVE_MAX_SPEED / Math.sqrt(absSpeed);
            speeds.vyMetersPerSecond *= Constants.DRIVE_MAX_SPEED / Math.sqrt(absSpeed);
        }
        if (Math.abs(speeds.omegaRadiansPerSecond) > Constants.DRIVE_MAX_ANGULAR_SPEED) {
            speeds.omegaRadiansPerSecond = Constants.DRIVE_MAX_ANGULAR_SPEED;
        }

        speeds.vyMetersPerSecond *= strafeSpeedScale.getAsDouble();
        speeds.omegaRadiansPerSecond *= turnSpeedScale.getAsDouble();

        MecanumDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);

        fl.setVoltage(flFeedForward.calculate(wheelSpeeds.frontLeftMetersPerSecond));
        fr.setVoltage(frFeedForward.calculate(wheelSpeeds.frontRightMetersPerSecond));
        bl.setVoltage(blFeedForward.calculate(wheelSpeeds.rearLeftMetersPerSecond));
        br.setVoltage(brFeedForward.calculate(wheelSpeeds.rearRightMetersPerSecond));
    }

    public void driveFieldOriented(ChassisSpeeds speeds) {
        double rotation = -getHeading().getRadians();
        double sin = Math.sin(rotation);
        double cos = Math.cos(rotation);

        double x = speeds.vxMetersPerSecond * cos - speeds.vyMetersPerSecond * sin;
        double y = speeds.vxMetersPerSecond * sin + speeds.vyMetersPerSecond * cos;

        speeds = new ChassisSpeeds(x, y, speeds.omegaRadiansPerSecond);

        drive(speeds);
    }

    public Command driveToPose(Pose2d pose) {
        PathConstraints constraints = new PathConstraints(
                Constants.DRIVE_MAX_SPEED,
                Constants.DRIVE_MAX_ACCELERATION,
                Constants.DRIVE_MAX_ANGULAR_SPEED,
                Constants.DRIVE_MAX_ANGULAR_ACCELERATION);

        return AutoBuilder.pathfindToPose(
                pose,
                constraints,
                0);
    }

    public Command alignToPoseFast(Pose2d pose) {
        return new AlignToPose(this, pose, 1);
    }

    public Command alignToPose(Pose2d pose) {
        return new AlignToPose(this, pose);
    }

    public Command alignToPose(Pose2d pose, int endCounts, double errorDist) {
        return new AlignToPose(this, pose, endCounts, errorDist);
    }

    public Command alignToPoleDeferred(Poles pole) {
        return new DeferredCommand(() -> alignToPose(vision.getPoleLocation(pole)), Set.of(this));

        // return new DeferredCommand(
        // () -> this.driveToPose(vision.getPoleLocation(pole)), Set.of(this));
    }

    public Command alignToClosestPole() {
        return new DeferredCommand(() -> alignToClosestPoleDeferred(), Set.of(this));
    }

    private Command alignToClosestPoleDeferred() {
        Pose2d polePosition = vision.getClosestPole();
        return alignToPose(polePosition);
    }

}
