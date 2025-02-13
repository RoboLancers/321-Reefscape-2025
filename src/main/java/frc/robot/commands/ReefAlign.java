/* (C) Robolancers 2025 */
package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.util.MyAlliance;
import frc.robot.util.ReefPosition;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class ReefAlign {
  /*
    Maps reef AprilTag ("tag") ID to left, center, and right alignment poses,
    loads only the tags on alliance reef as determined at robot initialization
  */
  public static final Map<Integer, Pose2d> leftAlignPoses = new HashMap<>();
  public static final Map<Integer, Pose2d> centerAlignPoses = new HashMap<>();
  public static final Map<Integer, Pose2d> rightAlignPoses = new HashMap<>();

  private static final Distance kMaxAlignDistance = Meter.of(2);
  private static final Distance kLeftAlignDistance = Inches.of(-6.5);
  private static final Distance kReefDistance = Inches.of(14);
  private static final Distance kRightAlignDistance = Inches.of(6.5);

  private static final Rotation2d kReefAlignmentRotation = Rotation2d.fromDegrees(180);
  private static final List<Integer> blueReefTagIDs = List.of(17, 18, 19, 20, 21, 22);
  private static final List<Integer> redReefTagIDs = List.of(6, 7, 8, 9, 10, 11);
  private static final List<AprilTag> blueReefTags =
      RobotConstants.kAprilTagFieldLayout.getTags().stream()
          .filter(tag -> blueReefTagIDs.contains(tag.ID))
          .toList();
  private static final List<AprilTag> redReefTags =
      RobotConstants.kAprilTagFieldLayout.getTags().stream()
          .filter(tag -> redReefTagIDs.contains(tag.ID))
          .toList();

  public static final Pose2d kRedCenterAlignPos = new Pose2d(13, 4, new Rotation2d());
  public static final Pose2d kBlueCenterAlignPos = new Pose2d(4.457, 4, new Rotation2d());

  public static final Distance kMechanismDeadbandThreshold =
      Meters.of(2); // distance to trigger mechanism
  public static final Distance kMaxAlignmentDeadbandThreshold =
      Meters.of(5); // distance to trigger alignment

  /**
   * This method is run during Robot#autonomousInit() and Robot#teleopInit() to save computations,
   * only the poses on the alliance reef are loaded
   */
  public static void loadReefAlignmentPoses() {
    final List<Integer> tagIDsToLoad = MyAlliance.isRed() ? redReefTagIDs : blueReefTagIDs;

    for (Integer id : tagIDsToLoad) {
      leftAlignPoses.computeIfAbsent(id, ReefAlign::getNearestLeftAlign);
      centerAlignPoses.computeIfAbsent(id, ReefAlign::getNearestCenterAlign);
      rightAlignPoses.computeIfAbsent(id, ReefAlign::getNearestRightAlign);
    }
  }

  /**
   * Finds the pose of the nearest reef tag on the alliance reef
   *
   * @param robotPose the pose of the robot to find the nearest reef tag pose for
   * @return null if robot alliance is unknown, otherwise a valid reef tag pose
   */
  public static Pose2d getNearestReefPose(Pose2d robotPose) {
    // `Optional` means the Alliance may not exist yet, which must be handled to proceed
    final Optional<Alliance> alliance = DriverStation.getAlliance();

    if (alliance.isEmpty()) return null;

    final List<AprilTag> tagsToCheck =
        alliance.get().equals(Alliance.Red) ? redReefTags : blueReefTags;

    return robotPose.nearest(tagsToCheck.stream().map(tag -> tag.pose.toPose2d()).toList());
  }

  /**
   * Finds the ID of the nearest reef tag on the alliance reef
   *
   * @param robotPose the pose of the robot to find the nearest reef tag ID for
   * @return -1 if robot alliance is unknown, otherwise a valid reef tag ID
   */
  public static int getNearestReefID(Pose2d robotPose) {
    Pose2d nearest = getNearestReefPose(robotPose);

    // handle the null that getNearestReefPose() may return when robot alliance is unknown
    if (nearest == null) return -1;

    return RobotConstants.kAprilTagFieldLayout.getTags().stream()
        .filter(tag -> tag.pose.toPose2d().equals(nearest))
        .toList()
        .get(0)
        .ID;
  }

  /**
   * Intended for aligning to the reef for removing algae
   *
   * @param reefTagID a valid reef tag ID
   * @return null if no reef tag of the ID specified is found, otherwise a valid robot pose aligned
   *     with the center of the nearest reef tag
   */
  private static Pose2d getNearestCenterAlign(int reefTagID) {
    // `Optional` here means there may not be a tag with the specified ID, again must be handled
    Optional<Pose3d> tagPose = RobotConstants.kAprilTagFieldLayout.getTagPose(reefTagID);

    if (tagPose.isEmpty()) return null;

    Pose2d aprilTagPose = tagPose.get().toPose2d();

    Pose2d resultPose =
        aprilTagPose.plus(new Transform2d(kReefDistance, Meter.zero(), kReefAlignmentRotation));

    return resultPose;
  }

  /**
   * Intended for aligning to the left side reef bars for scoring coral
   *
   * @param reefTagID a valid reef tag ID
   * @return null if no reef tag of the ID specified is found, otherwise a valid robot pose aligned
   *     with the center of the nearest reef tag
   */
  private static Pose2d getNearestLeftAlign(int reefTagID) {
    Optional<Pose3d> tagPose = RobotConstants.kAprilTagFieldLayout.getTagPose(reefTagID);

    if (tagPose.isEmpty()) return null;

    Pose2d aprilTagPose = tagPose.get().toPose2d();

    Pose2d resultPose =
        aprilTagPose.plus(
            new Transform2d(kReefDistance, kLeftAlignDistance, kReefAlignmentRotation));

    return resultPose;
  }

  /**
   * Intended for aligning to the right side reef bars for scoring coral
   *
   * @param reefTagID a valid reef tag ID
   * @return null if no reef tag of the ID specified is found, otherwise a valid robot pose aligned
   *     with the center of the nearest reef tag
   */
  private static Pose2d getNearestRightAlign(int reefTagID) {
    Optional<Pose3d> tagPose = RobotConstants.kAprilTagFieldLayout.getTagPose(reefTagID);

    if (tagPose.isEmpty()) return null;

    Pose2d aprilTagPose = tagPose.get().toPose2d();

    Pose2d resultPose =
        aprilTagPose.plus(
            new Transform2d(kReefDistance, kRightAlignDistance, kReefAlignmentRotation));

    return resultPose;
  }

  public static Command alignToReef(
      SwerveDrive swerveDrive, Supplier<ReefPosition> targetReefPosition) {
    return swerveDrive.driveToFieldPose(
        () ->
            switch (targetReefPosition.get()) {
              case ALGAE -> centerAlignPoses.get(getNearestReefID(swerveDrive.getPose()));
              case LEFT -> leftAlignPoses.get(getNearestReefID(swerveDrive.getPose()));
              case RIGHT -> rightAlignPoses.get(getNearestReefID(swerveDrive.getPose()));
              default -> swerveDrive.getPose(); // more or less a no-op
            });
  }

  public static void alignToReef(SwerveDrive drive, ReefPosition position) {
    drive.driveToFieldPose(
        switch (position) {
          case ALGAE -> centerAlignPoses.get(getNearestReefID(drive.getPose()));
          case LEFT -> leftAlignPoses.get(getNearestReefID(drive.getPose()));
          case RIGHT -> rightAlignPoses.get(getNearestReefID(drive.getPose()));
          default -> drive.getPose(); // more or less a no-op
        });
  }

  /**
   * Drives to align against the center of the nearest reef face (un;ess it's more than
   * kMaxAlignDistance away), no manual driving
   */
  public static Command goToNearestCenterAlign(SwerveDrive swerveDrive) {
    final Pose2d targetAlignPose = centerAlignPoses.get(getNearestReefID(swerveDrive.getPose()));
    return swerveDrive
        .driveToFieldPose(() -> targetAlignPose)
        .unless(
            () ->
                swerveDrive.getPose().getTranslation().getDistance(targetAlignPose.getTranslation())
                    > kMaxAlignDistance.in(Meters));
  }

  /**
   * Drives to align against the left side reef bars of the nearest reef face (un;ess it's more than
   * kMaxAlignDistance away), no manual driving
   */
  public static Command goToNearestLeftAlign(SwerveDrive swerveDrive) {
    final Pose2d targetAlignPose = leftAlignPoses.get(getNearestReefID(swerveDrive.getPose()));
    return swerveDrive
        .driveToFieldPose(() -> targetAlignPose)
        .unless(
            () ->
                swerveDrive.getPose().getTranslation().getDistance(targetAlignPose.getTranslation())
                    > kMaxAlignDistance.in(Meters));
  }

  /**
   * Drives to align against the right side reef bars of the nearest reef face (un;ess it's more
   * than kMaxAlignDistance away), no manual driving
   */
  public static Command goToNearestRightAlign(SwerveDrive swerveDrive) {
    final Pose2d targetAlignPose = rightAlignPoses.get(getNearestReefID(swerveDrive.getPose()));
    return swerveDrive
        .driveToFieldPose(() -> targetAlignPose)
        .unless(
            () ->
                swerveDrive.getPose().getTranslation().getDistance(targetAlignPose.getTranslation())
                    > kMaxAlignDistance.in(Meters));
  }

  /** Maintain translational driving while rotating toward the nearest reef tag */
  public static Command rotateToNearestReefTag(
      SwerveDrive swerveDrive, DoubleSupplier x, DoubleSupplier y) {
    return swerveDrive.driveFixedHeading(
        x,
        y,
        () -> getNearestReefPose(swerveDrive.getPose()).getRotation().plus(kReefAlignmentRotation));
  }

  public static void rotateToNearestReefTag(SwerveDrive swerveDrive, double x, double y) {
    swerveDrive.driveFixedHeading(
        x, y, getNearestReefPose(swerveDrive.getPose()).getRotation().plus(kReefAlignmentRotation));
  }

  // if robot is within 2 meters of either red or blue reef, auto-align will NOT work
  public static boolean isWithinReefRange(SwerveDrive drive, Distance deadband) {
    Pose2d centerPos = MyAlliance.isRed() ? kRedCenterAlignPos : kBlueCenterAlignPos;
    double deadbandDistance =
        Math.hypot(
            drive.getPose().getX() - centerPos.getX(), drive.getPose().getY() - centerPos.getY());

    return deadbandDistance < deadband.in(Meters);
  }
}
