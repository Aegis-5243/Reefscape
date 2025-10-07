package frc.robot.mecanumdrive;

import static edu.wpi.first.units.Units.Inches;

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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.units.Unit;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.UtilFunctions;
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
                .idleMode(IdleMode.kBrake) 
                .openLoopRampRate(.2)
                .closedLoopRampRate(.2);
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

        kinematics = new MecanumDriveKinematics(new Translation2d(0.259, 0.282),
                new Translation2d(0.259, -0.282),
                new Translation2d(-0.259, 0.282), new Translation2d(-0.259, -0.282));

        poseEstimator = new MecanumDrivePoseEstimator(kinematics, getRawHeading(),
                new MecanumDriveWheelPositions(),
                Pose2d.kZero);

        setUpPathPlanner();
        robotObject = field.getRobotObject();

        Shuffleboard.getTab("Teleoperated").add(field);

        ShuffleboardTab tab = Shuffleboard.getTab("Drive");
        tab.add("DriveSubsystem", this);
        tab.addFloatArray("Encoder Velocities", () -> {
            return new float[] {
                    (float) fl.getEncoder().getVelocity(),
                    (float) fr.getEncoder().getVelocity(),
                    (float) bl.getEncoder().getVelocity(),
                    (float) br.getEncoder().getVelocity()
            };
        });
        tab.addFloatArray("Alternate Encoder Velocities", () -> {
            return new float[] {
                    (float) fl.getAlternateEncoder().getVelocity(),
                    (float) fr.getAlternateEncoder().getVelocity(),
                    (float) bl.getAlternateEncoder().getVelocity(),
                    (float) br.getAlternateEncoder().getVelocity()
            };
        });
        tab.addFloatArray("Encoder Positions", () -> {
            return new float[] {
                    (float) fl.getEncoder().getPosition(),
                    (float) fr.getEncoder().getPosition(),
                    (float) bl.getEncoder().getPosition(),
                    (float) br.getEncoder().getPosition()
            };
        });
        tab.addFloatArray("Alternate Encoder Positions", () -> {
            return new float[] {
                    (float) fl.getAlternateEncoder().getPosition(),
                    (float) fr.getAlternateEncoder().getPosition(),
                    (float) bl.getAlternateEncoder().getPosition(),
                    (float) br.getAlternateEncoder().getPosition()
            };
        });
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
        // Shuffleboard.getTab("Teleoperated").add("Mecanum Drive", new Sendable() {
        //     @Override
        //     public void initSendable(SendableBuilder builder) {
        //         builder.setSmartDashboardType("SwerveDrive");

        //         builder.addDoubleProperty("Front Left Angle",
        //                 () -> flEncoder.getVelocity() < 0 ? 180 : 0, null);
        //         builder.addDoubleProperty("Front Left Velocity",
        //                 () -> Math.abs(flEncoder.getVelocity()), null);

        //         builder.addDoubleProperty("Front Right Angle",
        //                 () -> frEncoder.getVelocity() < 0 ? 180 : 0, null);
        //         builder.addDoubleProperty("Front Right Velocity",
        //                 () -> Math.abs(frEncoder.getVelocity()), null);

        //         builder.addDoubleProperty("Back Left Angle",
        //                 () -> blEncoder.getVelocity() < 0 ? 180 : 0, null);
        //         builder.addDoubleProperty("Back Left Velocity", () -> Math.abs(blEncoder.getVelocity()),
        //                 null);

        //         builder.addDoubleProperty("Back Right Angle",
        //                 () -> brEncoder.getVelocity() < 0 ? 180 : 0, null);
        //         builder.addDoubleProperty("Back Right Velocity",
        //                 () -> Math.abs(brEncoder.getVelocity()), null);

        //         builder.addDoubleProperty("Robot Angle",
        //                 () -> Units.radiansToDegrees(getChassisSpeeds().omegaRadiansPerSecond),
        //                 null);
        //     }
        // });
        Shuffleboard.getTab("Teleoperated").addDouble("Velocity", () -> getVelocity());
        tab.addDouble("xSpeed", () -> getChassisSpeeds().vxMetersPerSecond);
        tab.addDouble("ySpeed", () -> getChassisSpeeds().vyMetersPerSecond);
        tab.addDouble("zRotation", () -> getChassisSpeeds().omegaRadiansPerSecond);
        tab.addDouble("xPosition", () -> getPose().getX());
        tab.addDouble("yPosition", () -> getPose().getY());
        tab.addDouble("Heading", () -> getHeading().getDegrees());
        tab.addDouble("Raw Heading", () -> getRawHeading().getDegrees());

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
        poseEstimator.update(getRawHeading(), new MecanumDriveWheelPositions(
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
                        new PIDConstants(
                                UtilFunctions.getSettingSub("Align/kP",
                                        Constants.DRIVE_kP).getAsDouble(),
                                UtilFunctions.getSettingSub("Align/kI",
                                        Constants.DRIVE_kI).getAsDouble(),
                                UtilFunctions.getSettingSub("Align/kD",
                                        Constants.DRIVE_kD).getAsDouble()),
                        new PIDConstants(
                                UtilFunctions.getSettingSub("Align/rkP",
                                        Constants.DRIVE_ROTATE_kP).getAsDouble()
                                        * Math.PI / 180,
                                UtilFunctions.getSettingSub("Align/rkI",
                                        Constants.DRIVE_ROTATE_kI).getAsDouble()
                                        * Math.PI / 180,
                                UtilFunctions.getSettingSub("Align/rkD",
                                        Constants.DRIVE_ROTATE_kD).getAsDouble()
                                        * Math.PI / 180)),
                config,
                () -> {
                    return false;
                    // Optional<Alliance> alliance = DriverStation.getAlliance();
                    // if (alliance.isPresent()) {
                    //     return alliance.get() == DriverStation.Alliance.Red;
                    // }
                    // return false;
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
        return poseEstimator.getEstimatedPosition().getRotation();
    }
    
    public Rotation2d getRawHeading() {
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
            speeds.omegaRadiansPerSecond = Constants.DRIVE_MAX_ANGULAR_SPEED
                    * Math.signum(speeds.omegaRadiansPerSecond);
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

    public Command driveToPoseLoose(Pose2d pose) {
        return driveToPoseLoose(pose, 3.0, 30.0);
    }

    public Command driveToPoseLoose(Pose2d pose, double distanceTolerance, double degreeTolerance) {
        return driveToPose(pose).until(() -> {
            Pose2d currentPose = getPose();

            Transform2d trans = pose.minus(currentPose);
            double degrees = Math.abs(trans.getRotation().getDegrees());
            
            double distX = trans.getMeasureX().in(Inches);
            double distY = trans.getMeasureY().in(Inches);

            double dist = distX * distX + distY * distY;

            return (dist < distanceTolerance * distanceTolerance) && degrees < degreeTolerance;
        });
    }

    public Command driveToPose(Pose2d pose) {
        return driveToPose(pose, 0);
    }

    public Command driveToPose(Pose2d pose, double endVelocity) {
        PathConstraints constraints = new PathConstraints(
                Constants.DRIVE_MAX_SPEED,
                Constants.DRIVE_MAX_ACCELERATION,
                Constants.DRIVE_MAX_ANGULAR_SPEED,
                Constants.DRIVE_MAX_ANGULAR_ACCELERATION);

        return AutoBuilder.pathfindToPose(
                pose,
                constraints,
                endVelocity);
    }

    public Command alignToPoseFast(Pose2d pose) {
        return new AlignToPose(this, pose, 1);
    }

    public Command alignToPose(Pose2d pose) {
        return new AlignToPoseRobotCentric(this, pose);
    }

    public Command alignToPose(Pose2d pose, int endCounts, double errorDist) {
        return new AlignToPoseRobotCentric(this, pose, endCounts, errorDist);
    }

    public Command alignToPoleDeferred(Poles pole) {
        return new DeferredCommand(() -> alignToPose(vision.getPoleLocation(pole)), Set.of(this));

        // return new DeferredCommand(
        // () -> this.driveToPose(vision.getPoleLocation(pole)), Set.of(this));
    }

    public Command fineDriveToClosestPole() {
        return fineDriveToClosestPole(0);
    }

    public Command fineDrivetoPole(Poles pole) {
        return fineDriveToPole(pole, -0.08);
    }

    public Command fineDriveToPole(Poles pole, double offset) {
        return new DeferredCommand(() -> fineDriveToPoleDeferred(pole, offset), Set.of(this));
    }

    /** Offset of zero means bumpers will be touching the reef
     * @param offset meters away from reef
     */
    public Command fineDriveToClosestPole(double offset) {
        return new DeferredCommand(() -> fineDriveToClosestPoleDeferred(offset), Set.of(this));
    }

    private Command fineDriveToPoleDeferred(Poles pole, double offset) {
        Pose2d polePosition = vision.getPoleLocation(pole);
        return driveToPoseLoose(polePosition.transformBy(new Transform2d(-0.2 - offset, 0, Rotation2d.kZero)))
                .andThen(alignToPose(polePosition.transformBy(new Transform2d(-offset, 0, Rotation2d.kZero))));
    }

    private Command fineDriveToClosestPoleDeferred(double offset) {
        Pose2d polePosition = vision.getClosestPole();
        return driveToPoseLoose(polePosition.transformBy(new Transform2d(-0.1 - offset, 0, Rotation2d.kZero)))
                .andThen(alignToPose(polePosition.transformBy(new Transform2d(-offset, 0, Rotation2d.kZero))));
    }

    public Command driveToClosestCoralSupply() {
        return new DeferredCommand(() -> driveToClosestCoralSupplyDeferred(), Set.of(this)).withName("Drive to supply deferred");
    }

    private Command driveToClosestCoralSupplyDeferred() {
        Pose2d coralPose = vision.getClosestCoralSupplyPoint();
        return driveToPose(coralPose);
    }

    public Command fineDriveToClosestCoralSupply() {
        return new DeferredCommand(this::fineDriveToClosestCoralSupplyDeferred, Set.of(this));
    }

    public Command fineDriveToClosestCoralSupplyDeferred() {
        Pose2d coralPose = vision.getClosestCoralSupplyPoint();
        return driveToPose(coralPose.transformBy(new Transform2d(0.1, 0, Rotation2d.kZero)), 0.1)
                .andThen(alignToPose(coralPose));
    }

}
