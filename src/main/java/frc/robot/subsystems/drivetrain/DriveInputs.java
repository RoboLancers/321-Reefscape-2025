/* (C) Robolancers 2025 */
package frc.robot.subsystems.drivetrain;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

@Logged
public class DriveInputs {
  // module inputs
  public SwerveModuleState[] moduleStates;
  public SwerveModuleState[] moduleTargetStates;

  // drivetrain inputs
  public ChassisSpeeds chassisSpeeds;
  public Pose2d pose;

  // gyro inputs
  public Rotation2d yaw;
  public Rotation2d pitch;
  public Rotation2d roll;
}
