/* (C) Robolancers 2025 */
package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

@Logged
public interface DriveIO {
  default void updateInputs(DriveInputs inputs) {}

  default void resetPose(Pose2d pose) {}

  default void setControl(SwerveRequest request) {}

  default void setSimControl(ChassisSpeeds speeds) {}

  default void addVisionMeasurement(Pose2d visionPoseEstimate, double timestamp) {}
}
