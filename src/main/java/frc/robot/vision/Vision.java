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
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.mecanumdrive.DriveSubsystem;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.UtilFunctions;

/** Covers vision and odometry positioning */
public class Vision extends SubsystemBase {
    public static AprilTagFieldLayout fieldLayout = AprilTagFieldLayout
            .loadField(AprilTagFields.k2025ReefscapeAndyMark);

    DriveSubsystem driveSubsystem;

    private HashMap<Poles, Pose2d> bluePoses = new HashMap<Vision.Poles, Pose2d>();

    private HashMap<Algae, Pose2d> blueAlgae = new HashMap<Vision.Algae, Pose2d>();

    private BooleanSupplier isCoralSupplier;
    private boolean aprilTagAllowed;

    public Vision(DriveSubsystem driveSubsystem) {
        super();

        this.driveSubsystem = driveSubsystem;

        // get blue poles
        Transform2d reefCenter = new Transform2d(4.5, 4, Rotation2d.fromDegrees(0));
        double distFromCenter = 1.370;
        Pose2d offsetLeft = new Pose2d(-distFromCenter, Units.inchesToMeters(6.5), Rotation2d.fromDegrees(0));
        Pose2d offsetMiddle = new Pose2d(-distFromCenter, 0, Rotation2d.fromDegrees(0));
        Pose2d offsetRight = new Pose2d(-distFromCenter, -Units.inchesToMeters(6.5), Rotation2d.fromDegrees(0));

        bluePoses.put(Poles.PoleA, offsetLeft.rotateBy(Rotation2d.fromDegrees(0)).plus(reefCenter));
        bluePoses.put(Poles.PoleB, offsetRight.rotateBy(Rotation2d.fromDegrees(0)).plus(reefCenter));
        bluePoses.put(Poles.PoleC, offsetLeft.rotateBy(Rotation2d.fromDegrees(60)).plus(reefCenter));
        bluePoses.put(Poles.PoleD, offsetRight.rotateBy(Rotation2d.fromDegrees(60)).plus(reefCenter));
        bluePoses.put(Poles.PoleE, offsetLeft.rotateBy(Rotation2d.fromDegrees(120)).plus(reefCenter));
        bluePoses.put(Poles.PoleF, offsetRight.rotateBy(Rotation2d.fromDegrees(120)).plus(reefCenter));
        bluePoses.put(Poles.PoleG, offsetLeft.rotateBy(Rotation2d.fromDegrees(180)).plus(reefCenter));
        bluePoses.put(Poles.PoleH, offsetRight.rotateBy(Rotation2d.fromDegrees(180)).plus(reefCenter));
        bluePoses.put(Poles.PoleI, offsetLeft.rotateBy(Rotation2d.fromDegrees(240)).plus(reefCenter));
        bluePoses.put(Poles.PoleJ, offsetRight.rotateBy(Rotation2d.fromDegrees(240)).plus(reefCenter));
        bluePoses.put(Poles.PoleK, offsetLeft.rotateBy(Rotation2d.fromDegrees(300)).plus(reefCenter));
        bluePoses.put(Poles.PoleL, offsetRight.rotateBy(Rotation2d.fromDegrees(300)).plus(reefCenter));

        blueAlgae.put(Algae.AlgaeAB, offsetMiddle.rotateBy(Rotation2d.fromDegrees(0)).plus(reefCenter));
        blueAlgae.put(Algae.AlgaeCD, offsetMiddle.rotateBy(Rotation2d.fromDegrees(60)).plus(reefCenter));
        blueAlgae.put(Algae.AlgaeEF, offsetMiddle.rotateBy(Rotation2d.fromDegrees(120)).plus(reefCenter));
        blueAlgae.put(Algae.AlgaeGH, offsetMiddle.rotateBy(Rotation2d.fromDegrees(180)).plus(reefCenter));
        blueAlgae.put(Algae.AlgaeIJ, offsetMiddle.rotateBy(Rotation2d.fromDegrees(240)).plus(reefCenter));
        blueAlgae.put(Algae.AlgaeKL, offsetMiddle.rotateBy(Rotation2d.fromDegrees(360)).plus(reefCenter));

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
        boolean useMegaTag2 = true; // set to false to use MegaTag1
        boolean doRejectUpdate = false;

        if (aprilTagAllowed) {
            if (useMegaTag2 == false) {
                LimelightHelpers.PoseEstimate mt1 = LimelightHelpers
                        .getBotPoseEstimate_wpiBlue(Constants.FRONT_LIMELIGHT);

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
                LimelightHelpers.SetRobotOrientation(Constants.FRONT_LIMELIGHT,
                        driveSubsystem.poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
                LimelightHelpers.PoseEstimate mt2 = LimelightHelpers
                        .getBotPoseEstimate_wpiBlue_MegaTag2(Constants.FRONT_LIMELIGHT);
                if (Math.abs(driveSubsystem.gyro.getRate()) > 720) // if our angular velocity is greater than 720
                                                                   // degrees
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
        }
    };

    public void addCoralModeSupplier(BooleanSupplier coralMode) {
        isCoralSupplier = coralMode;
    }

    public HashMap<Poles, Pose2d> getPoles(boolean red) {
        return bluePoses;
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

    public Pose2d getPoleLocation(Poles pole) {
        return bluePoses.get(pole);
    }

}
