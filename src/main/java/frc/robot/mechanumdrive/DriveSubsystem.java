package frc.robot.mechanumdrive;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
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

import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {

    public SparkMax fl;
    public SparkMax fr;
    public SparkMax bl;
    public SparkMax br;

    public RelativeEncoder flEncoder;
    public RelativeEncoder frEncoder;
    public RelativeEncoder blEncoder;
    public RelativeEncoder brEncoder;

    public MecanumDrive mechanum;

    public AHRS gyro;

    public MecanumDriveKinematics kinematics;

    public MecanumDrivePoseEstimator poseEstimator;

    public Field2d field;

    public DriveSubsystem() {
        fl = new SparkMax(Constants.FL, MotorType.kBrushless);
        fr = new SparkMax(Constants.FR, MotorType.kBrushless);
        bl = new SparkMax(Constants.BL, MotorType.kBrushless);
        br = new SparkMax(Constants.BR, MotorType.kBrushless);

        SparkBaseConfig motorConfig = new SparkMaxConfig()
                .apply(new AlternateEncoderConfig()
                        .positionConversionFactor(Constants.MECHANUM_ALTERNATE_POSITION_CONVERSION_FACTOR)
                        .velocityConversionFactor(Constants.MECHANUM_ALTERNATE_VELOCITY_CONVERSION_FACTOR))
                .apply(
                        new EncoderConfig()
                                .positionConversionFactor(Constants.MECHANUM_POSITION_CONVERSION_FACTOR)
                                .velocityConversionFactor(Constants.MECHANUM_VELOCITY_CONVERSION_FACTOR))
                .idleMode(IdleMode.kCoast);

        // Match inversions and apply global config
        fl.configure(new SparkMaxConfig()
                .inverted(false)
                .apply(motorConfig), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        fr.configure(new SparkMaxConfig()
                .inverted(false)
                .apply(motorConfig), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        bl.configure(new SparkMaxConfig()
                .inverted(false)
                .apply(motorConfig), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        br.configure(new SparkMaxConfig()
                .inverted(false)
                .apply(motorConfig), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Able to substitute since conversion factors match gearbox ratio of 12.75
        flEncoder = fl.getEncoder();
        frEncoder = fr.getAlternateEncoder();
        blEncoder = bl.getAlternateEncoder();
        brEncoder = br.getAlternateEncoder();

        mechanum = new MecanumDrive(fl, fr, bl, br);

        kinematics = new MecanumDriveKinematics(new Translation2d(0.259, 0.283), new Translation2d(0.259, -0.283),
                new Translation2d(-0.259, 0.283), new Translation2d(-0.259, -0.283));

        poseEstimator = new MecanumDrivePoseEstimator(kinematics, getHeading(), new MecanumDriveWheelPositions(),
                Pose2d.kZero);

        field = new Field2d();

        SmartDashboard.putData("field", field);

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
                this::setPose,
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

        ShuffleboardTab tab = Shuffleboard.getTab("Drive");
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
    }

    @Override
    public void periodic() {
        updatePoseEstimate();
    }

    public void setUpPathPlanner() {

    }

    public void drive(ChassisSpeeds speeds) {

    }

    public ChassisSpeeds getChassisSpeeds() {
        return new ChassisSpeeds();
    }

    public Rotation2d getHeading() {
        return gyro.getRotation2d();
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void setPose(Pose2d pose) {
        poseEstimator.resetPose(pose);
    }

    public void updatePoseEstimate() {

    }
}
