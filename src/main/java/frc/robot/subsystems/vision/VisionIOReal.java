/* (C) Robolancers 2025 */
package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.subsystems.vision.VisionConstants.CameraConfig;
import java.util.List;
import java.util.function.BiConsumer;
import java.util.stream.Stream;
import org.photonvision.EstimatedRobotPose;

public class VisionIOReal implements VisionIO {
  private final List<PoseEstimatorCamera> cameras;

  public VisionIOReal(CameraConfig... configs) {
    this.cameras = Stream.of(configs).map(PoseEstimatorCamera::createReal).toList();
  }

  @Override
  public void updateEstimates(BiConsumer<EstimatedRobotPose, Matrix<N3, N1>> visionDataConsumer) {
    cameras.forEach(PoseEstimatorCamera::update);

    for (final var visionEst :
        cameras.stream().map(PoseEstimatorCamera::getLatestEstimate).toList()) {
      visionDataConsumer.accept(
          visionEst.pose(), Vision.calculateStdDevs(visionEst.relevantResult()));
    }
  }
}
