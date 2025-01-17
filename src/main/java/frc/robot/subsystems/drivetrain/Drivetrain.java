/* (C) Robolancers 2025 */
package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
// Import relevant classes.
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.File;
import java.io.IOException;
import java.util.function.DoubleSupplier;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;

// Example SwerveDrive class
public class Drivetrain extends SubsystemBase {
  SwerveDrive swerveDrive;

  public Drivetrain() throws IOException {
    double maximumSpeed = Units.feetToMeters(4.5);

    File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
    swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed);
  }

  /**
   * Command to drive the robot using translative values and heading as a setpoint.
   *
   * @param translationX Translation in the X direction.
   * @param translationY Translation in the Y direction.
   * @param headingX Heading X to calculate angle of the joystick.
   * @param headingY Heading Y to calculate angle of the joystick.
   * @return Drive command.
   */
  public Command driveCommand(
      DoubleSupplier translationX,
      DoubleSupplier translationY,
      DoubleSupplier headingX,
      DoubleSupplier headingY) {
    return run(
        () -> {
          Translation2d scaledInputs =
              SwerveMath.scaleTranslation(
                  new Translation2d(translationX.getAsDouble(), translationY.getAsDouble()), 0.8);

          // Make the robot move
          driveFieldOriented(
              swerveDrive.swerveController.getTargetSpeeds(
                  scaledInputs.getX(),
                  scaledInputs.getY(),
                  headingX.getAsDouble(),
                  headingY.getAsDouble(),
                  swerveDrive.getOdometryHeading().getRadians(),
                  swerveDrive.getMaximumChassisAngularVelocity()));
        });
  }

  private void driveFieldOriented(ChassisSpeeds targetSpeeds) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'driveFieldOriented'");
  }

  /**
   * Command to drive the robot using translative values and heading as angular velocity.
   *
   * @param translationX Translation in the X direction.
   * @param translationY Translation in the Y direction.
   * @param angularRotationX Rotation of the robot to set
   * @return Drive command.
   */
  public Command driveCommand(
      DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX) {
    return run(
        () -> {
          // Make the robot move
          swerveDrive.drive(
              new Translation2d(
                  translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                  translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()),
              angularRotationX.getAsDouble() * swerveDrive.getMaximumChassisAngularVelocity(),
              true,
              false);
        });
  }
}
