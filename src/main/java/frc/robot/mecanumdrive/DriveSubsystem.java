package frc.robot.mecanumdrive;

import java.util.Map;
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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
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

    public DriveSubsystem(Field2d field) {
        this.field = field;

        strafeSpeedScale = UtilFunctions.getSettingSub("mecanum/strafeSpeedScale", 1.22);
        turnSpeedScale = UtilFunctions.getSettingSub("mecanum/turnSpeedScale", 1.08);

        fl = new SparkMax(Constants.FL, MotorType.kBrushless);
        fr = new SparkMax(Constants.FR, MotorType.kBrushless);
        bl = new SparkMax(Constants.BL, MotorType.kBrushless);
        br = new SparkMax(Constants.BR, MotorType.kBrushless);

        SparkBaseConfig motorConfig = new SparkMaxConfig()
                .apply(new AlternateEncoderConfig()
                        .countsPerRevolution(8192)
                        .setSparkMaxDataPortConfig()
                        .positionConversionFactor(
                                Constants.MECANUM_ALTERNATE_POSITION_CONVERSION_FACTOR)
                        .velocityConversionFactor(
                                Constants.MECANUM_ALTERNATE_VELOCITY_CONVERSION_FACTOR))
                .apply(
                        new EncoderConfig()
                                .positionConversionFactor(
                                        Constants.MECANUM_POSITION_CONVERSION_FACTOR)
                                .velocityConversionFactor(
                                        Constants.MECANUM_VELOCITY_CONVERSION_FACTOR))
                .idleMode(IdleMode.kBrake) // TODO: test with loopRampRates
                .openLoopRampRate(.5)
                .closedLoopRampRate(.5);
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

        System.out.println(Constants.MECANUM_ALTERNATE_POSITION_CONVERSION_FACTOR);
        System.out.println(Constants.MECANUM_ALTERNATE_VELOCITY_CONVERSION_FACTOR);

        // Able to substitute since conversion factors match gearbox ratio of 12.75
        flEncoder = fl.getEncoder();
        frEncoder = fr.getEncoder();
        blEncoder = bl.getEncoder();
        brEncoder = br.getEncoder();

        fl.getEncoder().setPosition(0);
        fr.getEncoder().setPosition(0);
        bl.getEncoder().setPosition(0);
        br.getEncoder().setPosition(0);

        fl.getAlternateEncoder().setPosition(0);
        fr.getAlternateEncoder().setPosition(0);
        bl.getAlternateEncoder().setPosition(0);
        br.getAlternateEncoder().setPosition(0);

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
        robotObject = field.getRobotObject();

        Shuffleboard.getTab("Teleoperated").add(field)
                .withPosition(0, 0)
                .withSize(6, 3);

        ShuffleboardTab tab = Shuffleboard.getTab("Drive");
        tab.add("DriveSubsystem", this)
                .withPosition(0, 0)
                .withSize(3, 1);
        tab.addFloatArray("Encoder Velocities", () -> {
            return new float[] {
                    (float) fl.getEncoder().getVelocity(),
                    (float) fr.getEncoder().getVelocity(),
                    (float) bl.getEncoder().getVelocity(),
                    (float) br.getEncoder().getVelocity()
            };
        })
                .withPosition(0, 1)
                .withSize(2, 1);
        tab.addFloatArray("Alternate Encoder Velocities", () -> {
            return new float[] {
                    (float) fl.getAlternateEncoder().getVelocity(),
                    (float) fr.getAlternateEncoder().getVelocity(),
                    (float) bl.getAlternateEncoder().getVelocity(),
                    (float) br.getAlternateEncoder().getVelocity()
            };
        })
                .withPosition(0, 2)
                .withSize(2, 1);
        tab.addFloatArray("Encoder Positions", () -> {
            return new float[] {
                    (float) fl.getEncoder().getPosition(),
                    (float) fr.getEncoder().getPosition(),
                    (float) bl.getEncoder().getPosition(),
                    (float) br.getEncoder().getPosition()
            };
        })
                .withPosition(0, 3)
                .withSize(2, 1);
        tab.addFloatArray("Alternate Encoder Positions", () -> {
            return new float[] {
                    (float) fl.getAlternateEncoder().getPosition(),
                    (float) fr.getAlternateEncoder().getPosition(),
                    (float) bl.getAlternateEncoder().getPosition(),
                    (float) br.getAlternateEncoder().getPosition()
            };
        })
                .withPosition(0, 4)
                .withSize(2, 1);
        tab.addFloatArray("Motor Voltages", () -> {
            return new float[] {
                    (float) (fl.getBusVoltage() * fl.getAppliedOutput()),
                    (float) (fr.getBusVoltage() * fr.getAppliedOutput()),
                    (float) (bl.getBusVoltage() * bl.getAppliedOutput()),
                    (float) (br.getBusVoltage() * br.getAppliedOutput())
            };
        });

        // (future reference) Can be replaced with SmartDashboard.putData and manually
        // added to elastic
        // https://frc-elastic.gitbook.io/docs/additional-features-and-references/widgets-list-and-properties-reference
        Shuffleboard.getTab("Teleoperated").add("Mecanum Drive", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("SwerveDrive");

                builder.addDoubleProperty("Front Left Angle",
                        () -> flEncoder.getVelocity() < 0 ? 180 : 0, null);
                builder.addDoubleProperty("Front Left Velocity",
                        () -> Math.abs(flEncoder.getVelocity()), null);

                builder.addDoubleProperty("Front Right Angle",
                        () -> frEncoder.getVelocity() < 0 ? 180 : 0, null);
                builder.addDoubleProperty("Front Right Velocity",
                        () -> Math.abs(frEncoder.getVelocity()), null);

                builder.addDoubleProperty("Back Left Angle",
                        () -> blEncoder.getVelocity() < 0 ? 180 : 0, null);
                builder.addDoubleProperty("Back Left Velocity", () -> Math.abs(blEncoder.getVelocity()),
                        null);

                builder.addDoubleProperty("Back Right Angle",
                        () -> brEncoder.getVelocity() < 0 ? 180 : 0, null);
                builder.addDoubleProperty("Back Right Velocity",
                        () -> Math.abs(brEncoder.getVelocity()), null);

                builder.addDoubleProperty("Robot Angle",
                        () -> Units.radiansToDegrees(getChassisSpeeds().omegaRadiansPerSecond), null);
            }
        }).withPosition(2, 3)
                .withSize(2, 2);
        Shuffleboard.getTab("Teleoperated").addDouble("Velocity", () -> getVelocity())
                .withPosition(8, 0)
                .withSize(2, 2)
                .withWidget(BuiltInWidgets.kDial)
                .withProperties(Map.of("Min", 0, "Max", 8));
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

    private double getRelativeMovementAngle() {
        ChassisSpeeds speeds = getChassisSpeeds();
        double angle = Math.toDegrees(Math.atan2(speeds.vyMetersPerSecond, speeds.vxMetersPerSecond));
        return angle;
    }

    private double getVelocity() {
        ChassisSpeeds speeds = getChassisSpeeds();
        return Math.sqrt(speeds.vxMetersPerSecond * speeds.vxMetersPerSecond
                + speeds.vyMetersPerSecond * speeds.vyMetersPerSecond);
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
            config = null;
            e.printStackTrace();
        }

        AutoBuilder.configure(
                this::getPose,
                this::resetPose,
                this::getChassisSpeeds,
                this::drive,
                new PPHolonomicDriveController(
                        new PIDConstants(Constants.DRIVE_kP, Constants.DRIVE_kI, Constants.DRIVE_kD),
                        new PIDConstants(Constants.DRIVE_ROTATE_kP * Math.PI / 180, Constants.DRIVE_ROTATE_kI * Math.PI / 180, Constants.DRIVE_ROTATE_kD * Math.PI / 180)),
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
                angularRotationX.getAsDouble()))
                        .withName("driveCommandRobotCentric");
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
            speeds.omegaRadiansPerSecond = Constants.DRIVE_MAX_ANGULAR_SPEED * Math.signum(speeds.omegaRadiansPerSecond);
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
        double rotation = -poseEstimator.getEstimatedPosition().getRotation().getRadians();
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

    
    public Command alignToClosestCoralSupply() {
        return new DeferredCommand(() -> alignToClosestCoralSupplyDeferred(), Set.of(this));
    }

    private Command alignToClosestCoralSupplyDeferred() {
        Pose2d coralPose = vision.getClosestCoralSupplyPoint();
        return alignToPose(coralPose);
    }

}
