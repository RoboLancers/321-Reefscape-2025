/* (C) Robolancers 2025 */
package frc.robot.subsystems.vision;

import frc.robot.subsystems.vision.VisionConstants.CameraConfig;
import org.photonvision.PhotonCamera;

public class CameraIOReal implements CameraIO {
  private final PhotonCamera camera;

  public CameraIOReal(CameraConfig config) {
    this.camera = new PhotonCamera(config.cameraName());
  }

  @Override
  public void updateInputs(CameraInputs inputs) {
    // sidenote: in Java 21 updateInputs could be a one-liner with List#getLast
    final var unreadResults = camera.getAllUnreadResults();

    inputs.latestResult = unreadResults.get(unreadResults.size() - 1);
  }
}
