/* (C) Robolancers 2025 */
package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.function.BiConsumer;
import org.photonvision.EstimatedRobotPose;

public interface VisionIO {
  default void updateEstimates(BiConsumer<EstimatedRobotPose, Matrix<N3, N1>> visionDataConsumer) {}
}
