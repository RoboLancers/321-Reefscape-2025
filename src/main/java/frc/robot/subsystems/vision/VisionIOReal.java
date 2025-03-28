/* (C) Robolancers 2025 */
package frc.robot.subsystems.vision;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.vision.VisionConstants.CameraConfig;
import java.util.List;
import java.util.Objects;
import java.util.function.Supplier;
import java.util.stream.Stream;
import org.photonvision.PhotonCamera;

@Logged
public class VisionIOReal implements VisionIO {
  private final List<Camera> cameras;

  public VisionIOReal(Supplier<Rotation2d> robotHeadingSupplier, CameraConfig... configs) {
    cameras =
        Stream.of(configs)
            .map(
                config ->
                    new Camera(config, new PhotonCamera(config.cameraName()), robotHeadingSupplier))
            .toList();
  }

  @Override
  public VisionEstimate[] getLatestEstimates() {
    return cameras.stream()
        .map(Camera::tryLatestEstimate)
        .filter(Objects::nonNull)
        .toArray(VisionEstimate[]::new);
  }

  @Override
  public boolean reefCameraCanSeeReefTag(int tagID) {
    for (Camera camera : cameras) {
      if (!camera.isReefCamera()) continue;

      if (!camera.canSeeTag(tagID)) continue;

      return true;
    }

    return false;
  }

  @Override
  public boolean areCamerasConnected() {
    boolean isConnected = false;

    for (Camera camera : cameras) {
      isConnected = camera.isConnected();
      if (!isConnected) break;
    }

    return isConnected;
  }
}
