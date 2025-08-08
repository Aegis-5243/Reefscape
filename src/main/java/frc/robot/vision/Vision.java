package frc.robot.vision;

import java.util.HashMap;
import java.util.function.BooleanSupplier;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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

    private boolean doRejectUpdate = false;

    private Pose2d mt2pose;

    public Vision(DriveSubsystem driveSubsystem) {
        super();

        this.driveSubsystem = driveSubsystem;

        Shuffleboard.getTab("Teleoperated").addBoolean("IsUsingVision", () -> aprilTagAllowed);
        Shuffleboard.getTab("Teleoperated").addBoolean("IsSeeing", () -> !doRejectUpdate);

        LimelightHelpers.setPipelineIndex(Constants.FRONT_LIMELIGHT, 0);

        // get blue poles
        Transform2d reefCenter = new Transform2d(4.486, 4.045, Rotation2d.fromDegrees(0));
        /** The distance from the center of the reef to the middle of the bot
         * makes the bumpers touch the reef
         */
        double distFromCenter = 1.354;
        Pose2d offsetLeft = new Pose2d(-distFromCenter, Units.inchesToMeters(6.5), Rotation2d.fromDegrees(0));
        Pose2d offsetMiddle = new Pose2d(-distFromCenter, 0, Rotation2d.fromDegrees(0));
        Pose2d offsetRight = new Pose2d(-distFromCenter, -Units.inchesToMeters(6.5), Rotation2d.fromDegrees(0));

        bluePoses.put(Poles.PoleA, posePlusNoJunk(offsetLeft.rotateBy(Rotation2d.fromDegrees(0)), reefCenter));
        bluePoses.put(Poles.PoleB, posePlusNoJunk(offsetRight.rotateBy(Rotation2d.fromDegrees(0)), reefCenter));
        bluePoses.put(Poles.PoleC, posePlusNoJunk(offsetLeft.rotateBy(Rotation2d.fromDegrees(60)), reefCenter));
        bluePoses.put(Poles.PoleD, posePlusNoJunk(offsetRight.rotateBy(Rotation2d.fromDegrees(60)), reefCenter));
        bluePoses.put(Poles.PoleE, posePlusNoJunk(offsetLeft.rotateBy(Rotation2d.fromDegrees(120)), reefCenter));
        bluePoses.put(Poles.PoleF, posePlusNoJunk(offsetRight.rotateBy(Rotation2d.fromDegrees(120)), reefCenter));
        bluePoses.put(Poles.PoleG, posePlusNoJunk(offsetLeft.rotateBy(Rotation2d.fromDegrees(180)), reefCenter));
        bluePoses.put(Poles.PoleH, posePlusNoJunk(offsetRight.rotateBy(Rotation2d.fromDegrees(180)), reefCenter));
        bluePoses.put(Poles.PoleI, posePlusNoJunk(offsetLeft.rotateBy(Rotation2d.fromDegrees(240)), reefCenter));
        bluePoses.put(Poles.PoleJ, posePlusNoJunk(offsetRight.rotateBy(Rotation2d.fromDegrees(240)), reefCenter));
        bluePoses.put(Poles.PoleK, posePlusNoJunk(offsetLeft.rotateBy(Rotation2d.fromDegrees(300)), reefCenter));
        bluePoses.put(Poles.PoleL, posePlusNoJunk(offsetRight.rotateBy(Rotation2d.fromDegrees(300)), reefCenter));

        blueAlgae.put(Algae.AlgaeAB, posePlusNoJunk(offsetMiddle.rotateBy(Rotation2d.fromDegrees(0)), reefCenter));
        blueAlgae.put(Algae.AlgaeCD, posePlusNoJunk(offsetMiddle.rotateBy(Rotation2d.fromDegrees(60)), reefCenter));
        blueAlgae.put(Algae.AlgaeEF, posePlusNoJunk(offsetMiddle.rotateBy(Rotation2d.fromDegrees(120)), reefCenter));
        blueAlgae.put(Algae.AlgaeGH, posePlusNoJunk(offsetMiddle.rotateBy(Rotation2d.fromDegrees(180)), reefCenter));
        blueAlgae.put(Algae.AlgaeIJ, posePlusNoJunk(offsetMiddle.rotateBy(Rotation2d.fromDegrees(240)), reefCenter));
        blueAlgae.put(Algae.AlgaeKL, posePlusNoJunk(offsetMiddle.rotateBy(Rotation2d.fromDegrees(300)), reefCenter));

        // driveSubsystem.field.getObject("ScoringPoles").setPoses(bluePoses.values().toArray(Pose2d[]::new));
        // driveSubsystem.field.getObject("AlgaeLocs").setPoses(blueAlgae.values().toArray(Pose2d[]::new));

    }

    @Override
    public void periodic() {
        calcClosestPole();
        calcClosestCoralSupplyPoint();
        updateOdometry();

        driveSubsystem.field.getObject("closePole").setPose(closestPole);

        if (mt2pose == null) {
            driveSubsystem.field.getObject("mt2pose").setPose(new Pose2d(-1000, -1000, Rotation2d.kZero));
        } else {
            driveSubsystem.field.getObject("mt2est").setPose(mt2pose);
        }
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
        doRejectUpdate = false;

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

                if (mt2 == null) {
                    doRejectUpdate = true;
                    return;
                }

                mt2pose = mt2.pose;

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

    private Pose2d coralSupplyPoint = new Pose2d();

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

            for (var key : blueAlgae.keySet()) {
                var algaePose = blueAlgae.get(key);
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

    public void calcClosestCoralSupplyPoint() {

        double sgn = driveSubsystem.getPose().getY() > 4 ? 1 : -1;
        coralSupplyPoint = new Pose2d(1.187, 4 + 2.978 * sgn, Rotation2d.fromDegrees(-55 * sgn));
    }

    /**
     * Pose2d's .translateBy() and .plus() methods both use a relative translation
     * based on rotation, this function creates a global translation
     */
    public Pose2d posePlusNoJunk(Pose2d pose, Transform2d transform) {
        return new Pose2d(
                pose.getX() + transform.getX(),
                pose.getY() + transform.getY(),
                pose.getRotation());
    }

    public Pose2d getClosestPole() {
        return closestPole;
    }

    public Pose2d getClosestCoralSupplyPoint() {
        return coralSupplyPoint;
    }

    public void diableAprilTags() {
        aprilTagAllowed = false;
    }

    public void enableAprilTags() {
        aprilTagAllowed = true;
    }

    public Pose2d getPoleLocation(Poles pole) {
        return bluePoses.get(pole);
    }

}
