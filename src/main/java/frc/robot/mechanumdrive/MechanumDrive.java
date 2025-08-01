package frc.robot.mechanumdrive;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.studica.frc.AHRS;

import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class MechanumDrive extends SubsystemBase {

    public SparkMax fl;
    public SparkMax fr;
    public SparkMax bl;
    public SparkMax br;

    public MecanumDrive drive;

    public SlewRateLimiter limiter;

    public AHRS gyro;

    public MecanumDriveKinematics kinematics;

    public MecanumDrivePoseEstimator poseEstimator;

    public Field2d field;

    public MechanumDrive() {
        fl = new SparkMax(Constants.FL, MotorType.kBrushless);
        fr = new SparkMax(Constants.FR, MotorType.kBrushless);
        bl = new SparkMax(Constants.BL, MotorType.kBrushless);
        br = new SparkMax(Constants.BR, MotorType.kBrushless);

        kinematics = new MecanumDriveKinematics(new Translation2d(0.259, 0.283), new Translation2d(0.259, -0.283),
                new Translation2d(-0.259, 0.283), new Translation2d(-0.259, -0.283));

        field = new Field2d();
        
		SmartDashboard.putData("field", field);
    }

    public void drive(ChassisSpeeds speeds) {

    }
}
