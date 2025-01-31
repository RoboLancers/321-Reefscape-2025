/* (C) Robolancers 2025 */
package frc.robot.subsystems.drivetrain;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

@Logged
public class DriveIOIdeal implements DriveIO {
  public DriveIOIdeal() {}

  @Override
  public void updateInputs(DriveInputs inputs) {
    inputs.moduleStates = new SwerveModuleState[4];
    inputs.moduleTargetStates = new SwerveModuleState[4];
    inputs.chassisSpeeds = new ChassisSpeeds();
    inputs.pose = new Pose2d();

    inputs.yaw = new Rotation2d();
    inputs.pitch = new Rotation2d();
    inputs.roll = new Rotation2d();
  }
}
