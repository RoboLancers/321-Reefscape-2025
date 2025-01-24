/* (C) Robolancers 2025 */
package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

@Logged
public class DriveIOKraken extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements DriveIO {
  public DriveIOKraken(
      SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
    super(TalonFX::new, TalonFX::new, CANcoder::new, drivetrainConstants, modules);
    super.configNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void setControl(SwerveRequest request) {
    super.setControl(request);
  }

  @Override
  public void resetPose(Pose2d pose) {
    super.resetPose(pose);
  }

  @Override
  public void addVisionMeasurement(Pose2d visionEstimate, double timestamp) {
    super.addVisionMeasurement(visionEstimate, timestamp);
  }

  @Override
  public void updateInputs(DriveInputs inputs) {
    inputs.moduleStates = getState().ModuleStates;
    inputs.moduleTargetStates = getState().ModuleTargets;
    inputs.chassisSpeeds = getState().Speeds;
    inputs.pose = getState().Pose;

    inputs.yaw = Rotation2d.fromDegrees(getPigeon2().getYaw().getValueAsDouble());
    inputs.pitch = Rotation2d.fromDegrees(getPigeon2().getPitch().getValueAsDouble());
    inputs.roll = Rotation2d.fromDegrees(getPigeon2().getRoll().getValueAsDouble());
  }
}
