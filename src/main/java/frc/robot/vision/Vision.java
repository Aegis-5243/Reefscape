package frc.robot.vision;

import java.util.HashMap;
import java.util.Optional;
import java.util.function.BooleanSupplier;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.mecanumdrive.DriveSubsystem;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.UtilFunctions;

public class Vision extends SubsystemBase {
    public static AprilTagFieldLayout fieldLayout = AprilTagFieldLayout
            .loadField(AprilTagFields.k2025ReefscapeAndyMark);

    DriveSubsystem driveSubsystem;

    private HashMap<Poles, Pose2d> bluePoses = new HashMap<Vision.Poles, Pose2d>();
    private HashMap<Poles, Pose2d> redPoses = new HashMap<Vision.Poles, Pose2d>();

    private HashMap<Algae, Pose2d> blueAlgae = new HashMap<Vision.Algae, Pose2d>();

    private BooleanSupplier isCoralSupplier;
    private boolean aprilTagAllowed;

    public Vision(DriveSubsystem driveSubsystem) {
        super();

        this.driveSubsystem = driveSubsystem;

        // TODO: change coordinates to match different bot
        // get blue poles
        bluePoses.put(Poles.PoleA, new Pose2d(3.130, 4.190, Rotation2d.fromDegrees(0)));
        bluePoses.put(Poles.PoleB, new Pose2d(3.130, 3.862, Rotation2d.fromDegrees(0)));
        bluePoses.put(Poles.PoleC, new Pose2d(3.714, 3.004, Rotation2d.fromDegrees(60)));
        bluePoses.put(Poles.PoleD, new Pose2d(3.998, 2.839, Rotation2d.fromDegrees(60)));
        bluePoses.put(Poles.PoleE, new Pose2d(4.990, 2.839, Rotation2d.fromDegrees(120)));
        bluePoses.put(Poles.PoleF, new Pose2d(5.275, 3.003, Rotation2d.fromDegrees(120)));
        bluePoses.put(Poles.PoleG, new Pose2d(5.850, 3.849, Rotation2d.fromDegrees(180)));
        bluePoses.put(Poles.PoleH, new Pose2d(5.850, 4.177, Rotation2d.fromDegrees(180)));
        bluePoses.put(Poles.PoleI, new Pose2d(5.274, 5.048, Rotation2d.fromDegrees(-120)));
        bluePoses.put(Poles.PoleJ, new Pose2d(4.990, 5.213, Rotation2d.fromDegrees(-120)));
        bluePoses.put(Poles.PoleK, new Pose2d(3.999, 5.212, Rotation2d.fromDegrees(-60)));
        bluePoses.put(Poles.PoleL, new Pose2d(3.713, 5.049, Rotation2d.fromDegrees(-60)));

        blueAlgae.put(Algae.AlgaeAB, new Pose2d(3.104, 4.026, Rotation2d.fromDegrees(0)));
        blueAlgae.put(Algae.AlgaeCD, new Pose2d(3.799, 2.823, Rotation2d.fromDegrees(60)));
        blueAlgae.put(Algae.AlgaeEF, new Pose2d(5.190, 2.822, Rotation2d.fromDegrees(120)));
        blueAlgae.put(Algae.AlgaeGH, new Pose2d(5.883, 4.011, Rotation2d.fromDegrees(180)));
        blueAlgae.put(Algae.AlgaeIJ, new Pose2d(5.190, 5.230, Rotation2d.fromDegrees(-120)));
        blueAlgae.put(Algae.AlgaeKL, new Pose2d(3.799, 5.230, Rotation2d.fromDegrees(-60)));

        // get red poles
        for (var pole : bluePoses.keySet()) {
            var test = flipFieldAlways(bluePoses.get(pole));
            redPoses.put(pole, test);
        }

    }

    @Override
    public void periodic() {
        calcClosestPole();

        updateOdometry();
    }

    public enum Poles {
        PoleA, PoleB, PoleC, PoleD, PoleE, PoleF, PoleG, PoleH, PoleI, PoleJ, PoleK, PoleL
    }

    public enum Algae {
        AlgaeAB, AlgaeCD, AlgaeEF, AlgaeGH, AlgaeIJ, AlgaeKL
    }

    public Pose2d flipFieldAlways(Pose2d poseToFlip) {
        return poseToFlip.relativeTo(
                new Pose2d(
                        new Translation2d(fieldLayout.getFieldLength(), fieldLayout.getFieldWidth()),
                        new Rotation2d(Math.PI)));
    }

    public void updateOdometry() {
        MecanumDrivePoseEstimator estimator = driveSubsystem.poseEstimator;

        LimelightHelpers.SetRobotOrientation(Constants.FRONT_LIMELIGHT,
                driveSubsystem.getHeading().getDegrees(),
                0, 0, 0, 0, 0);

        boolean useMegaTag2 = true; // set to false to use MegaTag1
        boolean doRejectUpdate = false;
        if (useMegaTag2 == false) {
            LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");

            if (mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
                if (mt1.rawFiducials[0].ambiguity > .7) {
                    doRejectUpdate = true;
                }
                if (mt1.rawFiducials[0].distToCamera > 3) {
                    doRejectUpdate = true;
                }
            }
            if (mt1.tagCount == 0) {
                doRejectUpdate = true;
            }

            if (!doRejectUpdate) {
                driveSubsystem.poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));
                driveSubsystem.poseEstimator.addVisionMeasurement(
                        mt1.pose,
                        mt1.timestampSeconds);
            }
        } else if (useMegaTag2 == true) {
            LimelightHelpers.SetRobotOrientation("limelight",
                    driveSubsystem.poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
            LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
            if (Math.abs(driveSubsystem.gyro.getRate()) > 720) // if our angular velocity is greater than 720 degrees
                                                               // per second,
            // ignore vision updates
            {
                doRejectUpdate = true;
            }
            if (mt2.tagCount == 0) {
                doRejectUpdate = true;
            }
            if (!doRejectUpdate) {
                driveSubsystem.poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
                driveSubsystem.poseEstimator.addVisionMeasurement(
                        mt2.pose,
                        mt2.timestampSeconds);
            }
        }
    };

    public HashMap<Poles, Pose2d> getPoles(boolean red) {
        if (red) {
            return redPoses;
        } else {
            return bluePoses;
        }
    }

    private Pose2d closestPole = new Pose2d();

    private void calcClosestPole() {
        if (isCoralSupplier == null || isCoralSupplier.getAsBoolean()) {
            // find closest pole
            var poles = getPoles(driveSubsystem.isRedAlliance());
            var closePole = Poles.PoleA;
            var currentPose = driveSubsystem.getPose();
            var closeDist = UtilFunctions.getDistance(currentPose, poles.get(closePole));

            for (var key : poles.keySet()) {
                var newDist = UtilFunctions.getDistance(currentPose, poles.get(key));
                if (newDist < closeDist) {
                    closeDist = newDist;
                    closePole = key;
                }
            }

            closestPole = poles.get(closePole);
        } else {
            // find closest algae
            var closeAlgae = Algae.AlgaeAB;
            var currentPose = driveSubsystem.getPose();
            var finalPose = driveSubsystem.getPose();
            var closeDist = 10000000000.;
            var red = driveSubsystem.isRedAlliance();

            for (var key : blueAlgae.keySet()) {
                var algaePose = blueAlgae.get(key);
                if (red) {
                    algaePose = flipFieldAlways(algaePose);
                }
                var newDist = UtilFunctions.getDistance(currentPose, algaePose);
                if (newDist < closeDist) {
                    closeDist = newDist;
                    closeAlgae = key;
                    finalPose = algaePose;
                }
            }

            closestPole = finalPose;
        }

    }

    public Pose2d getClosestPole() {
        return closestPole;
    }

    public Command diableAprilTags() {
        return runOnce(
                () -> {
                    aprilTagAllowed = false;
                });
    }

    public Command enableAprilTags() {
        return runOnce(
                () -> {
                    aprilTagAllowed = true;
                });
    }

}
