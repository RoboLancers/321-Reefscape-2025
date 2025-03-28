/* (C) Robolancers 2025 */
package frc.robot.subsystems.vision;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.util.VirtualSubsystem;
import java.util.function.Consumer;
import java.util.function.Supplier;

@Logged
public class Vision extends VirtualSubsystem {
  private final VisionIO io;

  private final Consumer<VisionEstimate> visionDataConsumer;
  private final Consumer<VisionEstimate> reefVisionDataConsumer;

  public static Vision create(
      Supplier<Pose2d> robotPoseSupplier,
      Consumer<VisionEstimate> visionDataConsumer,
      Consumer<VisionEstimate> reefVisionDataConsumer,
      Supplier<Rotation3d> robotHeadingSupplier) {
    return RobotBase.isReal()
        ? new Vision(
            new VisionIOReal(robotHeadingSupplier, VisionConstants.kCameraConfigs),
            visionDataConsumer,
            reefVisionDataConsumer)
        : new Vision(
            new VisionIOSim(
                robotPoseSupplier, robotHeadingSupplier, VisionConstants.kCameraConfigs),
            visionDataConsumer,
            reefVisionDataConsumer);
  }

  private Vision(
      VisionIO io,
      Consumer<VisionEstimate> visionDataConsumer,
      Consumer<VisionEstimate> reefVisionDataConsumer) {
    this.io = io;
    this.visionDataConsumer = visionDataConsumer;
    this.reefVisionDataConsumer = reefVisionDataConsumer;
  }

  @Override
  public void periodic() {
    final var latestEstimates = io.getLatestEstimates();

    for (final var est : latestEstimates) {
      visionDataConsumer.accept(est);

      if (est.sourceType() == CameraUsage.REEF) {
        reefVisionDataConsumer.accept(est);
      }
    }
  }

  public boolean areCamerasConnected() {
    return io.areCamerasConnected();
  }
}
