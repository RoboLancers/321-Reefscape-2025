/* (C) Robolancers 2025 */
package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
// Import relevant classes.
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.File;
import java.io.IOException;
import java.util.function.DoubleSupplier;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

// Example SwerveDrive class
public class Drivetrain extends SubsystemBase {
  SwerveDrive swerveDrive;
  SwerveDrivePoseEstimator odometry;

  public Drivetrain() {
    double maximumSpeed = Units.feetToMeters(4.5);

    File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
    try {
      swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed);
    } catch (IOException e) {
      e.printStackTrace();
    }

    swerveDrive.resetOdometry(new Pose2d(1, 1, Rotation2d.fromDegrees(0)));
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    configureSwerve();
  }

  private void configureSwerve() {
    // swerveDrive.swerveController.setMaximumAngularVelocity(Constants.DrivetrainConstants.kMaxOmegaRadiansPerSecond);
    swerveDrive.setHeadingCorrection(
        false); // Heading correction should only be used while controlling the robot via angle.
    swerveDrive.setCosineCompensator(
        false); // !SwerveDriveTelemetry.isSimulation); // Disables cosine compensation for
    // simulations since it causes discrepancies not seen in real life.
    swerveDrive.setAngularVelocityCompensation(
        true, false,
        0.1); // Correct for skew that gets worse as angular velocity increases. Start with a
    // coefficient of 0.1.
    swerveDrive.setModuleEncoderAutoSynchronize(
        false, 1); // Enable if you want to resynchronize your absolute encoders and motor encoders
    // periodically when they are not moving.
    // swerveDrive.setChassisDiscretization(true, false,
    // Constants.DrivetrainConstants.kSecondOrderKinematicsDt);
    // swerveDrive
    //     .pushOffsetsToEncoders(); // Set the absolute encoder to be used over the internal
    // encoder
    // and push the offsets onto it. Throws warning if not possible
    swerveDrive.setModuleEncoderAutoSynchronize(false, 0);
  }

  public SwerveModulePosition[] getSwerveModulePosition() {
    return swerveDrive.getModulePositions();
  }

  public Pose2d getPose() {
    return odometry.getEstimatedPosition();
  }

  public SwerveModuleState[] getSwerveModuleState() {
    return swerveDrive.getStates();
  }

  public void addVisionMeasurement(
      Pose2d robotPose2d, double timestamp, Matrix<N3, N1> visionMeasurementStdDevs) {
    odometry.addVisionMeasurement(robotPose2d, timestamp, visionMeasurementStdDevs);
  }

  /**
   * Command to drive the robot using translative values and heading as angular velocity.
   *
   * @param translationX Translation in the X direction.
   * @param translationY Translation in the Y direction.
   * @param angularRotationX Rotation of the robot to set
   * @return Drive command.
   */
  public Command driveFieldRelativeCommand(
      DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX) {
    return run(
        () -> {
          // Make the robot move FIELD CENTRIC
          swerveDrive.drive(
              new Translation2d(
                  translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                  translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()),
              angularRotationX.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity(),
              true,
              false);
        });
  }

  /**
   * Command to drive the robot using translative values and heading as angular velocity.
   *
   * @param translationX Translation in the X direction.
   * @param translationY Translation in the Y direction.
   * @param angularRotationX Rotation of the robot to set
   * @return Drive command.
   */
  public Command driveRobotRelativeCommand(
      DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX) {
    return run(
        () -> {
          // Make the robot move ROBOT-CENTRIC
          swerveDrive.drive(
              new Translation2d(
                  translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                  translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()),
              angularRotationX.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity(),
              false,
              false);
        });
  }
}
