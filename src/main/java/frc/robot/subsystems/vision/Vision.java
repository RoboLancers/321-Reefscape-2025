/* (C) Robolancers 2025 */
package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.util.VirtualSubsystem;
import java.util.function.BiConsumer;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;

public class Vision extends VirtualSubsystem {
  private final VisionIO io;

  private final BiConsumer<EstimatedRobotPose, Matrix<N3, N1>> visionDataConsumer;

  public static Vision create(
      Supplier<Pose2d> robotPoseSupplier,
      BiConsumer<EstimatedRobotPose, Matrix<N3, N1>> visionDataConsumer) {
    return RobotBase.isReal()
        ? new Vision(new VisionIOReal(), visionDataConsumer)
        : new Vision(
            new VisionIOSim(robotPoseSupplier, VisionConstants.k427CameraConfigs),
            visionDataConsumer);
  }

  private Vision(VisionIO io, BiConsumer<EstimatedRobotPose, Matrix<N3, N1>> visionDataConsumer) {
    this.io = io;

    this.visionDataConsumer = visionDataConsumer;
  }

  @Override
  public void periodic() {
    io.updateEstimates(visionDataConsumer);
  }

  // could be absolute nonsense, open to tuning constants for each robot camera config
  public static Matrix<N3, N1> calculateStdDevs(PhotonPipelineResult result) {
    final double stdDev = result.metadata.getLatencyMillis() / result.getTargets().size();

    final double rotationStdDev = VisionConstants.kRotationStdDevCoeff * stdDev;

    final Translation3d robotPose = new Translation3d();

    // weighted average by ambiguity
    final double avgTargetDistance =
        result.getTargets().stream()
                .mapToDouble(
                    t -> {
                      final double bestCameraToTargetDistance =
                          t.getBestCameraToTarget().getTranslation().getDistance(robotPose);
                      final double altCameraToTargetDistance =
                          t.getAlternateCameraToTarget().getTranslation().getDistance(robotPose);

                      return t.getPoseAmbiguity()
                          * (bestCameraToTargetDistance + altCameraToTargetDistance);
                    })
                .reduce(0.0, Double::sum)
            / (2.0 * result.getTargets().size());

    final double translationStdDev =
        VisionConstants.kTranslationStdDevCoeff * avgTargetDistance * stdDev;

    return VecBuilder.fill(translationStdDev, translationStdDev, rotationStdDev);
  }
}
