/* (C) Robolancers 2025 */
package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import java.util.Set;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionTargetSim;

public class CameraIOSim implements CameraIO {
  private final PhotonCameraSim camera;
  private final Supplier<Pose3d> cameraPoseSupplier;
  private final Set<VisionTargetSim> targets;

  public CameraIOSim(
      PhotonCameraSim sim, Supplier<Pose3d> cameraPoseSupplier, Set<VisionTargetSim> targets) {
    this.camera = sim;
    this.cameraPoseSupplier = cameraPoseSupplier;
    this.targets = targets;
  }

  @Override
  public void updateInputs(CameraInputs inputs) {
    inputs.latestResult =
        camera.process(
            camera.prop.estLatencyMs(), cameraPoseSupplier.get(), targets.stream().toList());
  }
}
