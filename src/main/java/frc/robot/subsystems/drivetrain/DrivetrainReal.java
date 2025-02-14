/* (C) Robolancers 2025 */
package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.ModuleRequest;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/*
 * Real Drivetrain using CTRE SwerveDrivetrain and SwerveRequests
 */

@Logged
public class DrivetrainReal extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder>
    implements SwerveDrive {
  private final SwerveRequest.FieldCentric fieldCentricRequest =
      new SwerveRequest.FieldCentric()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withDesaturateWheelSpeeds(true);

  private final SwerveRequest.ApplyRobotSpeeds robotCentricRequest =
      new SwerveRequest.ApplyRobotSpeeds()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withDesaturateWheelSpeeds(true);

  private final SwerveRequest.FieldCentricFacingAngle fieldCentricFacingAngleRequest =
      new SwerveRequest.FieldCentricFacingAngle()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withDesaturateWheelSpeeds(true);

  private Pose2d alignmentSetpoint = Pose2d.kZero;

  public DrivetrainReal(
      SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
    // create CTRE Swervedrivetrain
    super(TalonFX::new, TalonFX::new, CANcoder::new, drivetrainConstants, modules);
    configNeutralMode(NeutralModeValue.Brake);
    configureAutoBuilder();
    configurePoseControllers();
  }

  @Override
  public Command teleopDrive(
      DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation) {
    return run(
        () -> {
          var speeds =
              ChassisSpeeds.discretize(
                  translationX.getAsDouble(),
                  translationY.getAsDouble(),
                  rotation.getAsDouble(),
                  DrivetrainConstants.kLoopDt.in(Seconds));

          // x braking
          // if(Math.abs(newTranslationX) < DriveConstants.kDriveDeadband &&
          // Math.abs(newTranslationY) < DriveConstants.kDriveDeadband &&
          // Math.abs(newRotation) < DriveConstants.kRotationDeadband){
          // setControl(new SwerveRequest.SwerveDriveBrake())};

          setControl(
              fieldCentricRequest
                  .withVelocityX(speeds.vxMetersPerSecond)
                  .withVelocityY(speeds.vyMetersPerSecond)
                  .withRotationalRate(speeds.omegaRadiansPerSecond)
                  .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective));
        });
  }

  @Override
  public Command teleopDriveFixedHeading(
      DoubleSupplier translationX,
      DoubleSupplier translationY,
      DoubleSupplier rotationX,
      DoubleSupplier rotationY) {
    return run(
        () -> {
          Rotation2d desiredRotation =
              flipRotation(
                  Rotation2d.fromRadians(
                      Math.atan2(rotationX.getAsDouble(), rotationY.getAsDouble())));

          var speeds =
              ChassisSpeeds.discretize(
                  translationX.getAsDouble(),
                  translationY.getAsDouble(),
                  0,
                  DrivetrainConstants.kLoopDt.in(Seconds));

          setControl(
              fieldCentricFacingAngleRequest
                  .withDriveRequestType(DriveRequestType.Velocity)
                  .withVelocityX(speeds.vxMetersPerSecond)
                  .withVelocityY(speeds.vyMetersPerSecond)
                  .withTargetDirection(desiredRotation)
                  .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective));
        });
  }

  @Override
  public Command driveFieldCentric(
      DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation) {
    return run(
        () -> {
          var speeds =
              ChassisSpeeds.discretize(
                  translationX.getAsDouble(),
                  translationY.getAsDouble(),
                  rotation.getAsDouble(),
                  DrivetrainConstants.kLoopDt.in(Seconds));

          setControl(
              fieldCentricRequest
                  .withVelocityX(speeds.vxMetersPerSecond)
                  .withVelocityY(speeds.vyMetersPerSecond)
                  .withRotationalRate(speeds.omegaRadiansPerSecond));
        });
  }
  ;

  @Override
  public Command driveRobotCentric(
      DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation) {
    return run(
        () ->
            driveRobotCentric(
                translationX.getAsDouble(),
                translationY.getAsDouble(),
                rotation.getAsDouble(),
                DriveFeedforwards.zeros(4)));
  }

  @Override
  public void driveRobotCentric(
      double translationX, double translationY, double rotation, DriveFeedforwards feedforwards) {
    // var speeds =
    //     ChassisSpeeds.discretize(
    //         translationX, translationY, rotation, DrivetrainConstants.kLoopDt.in(Seconds));

    var speeds = new ChassisSpeeds(translationX, translationY, rotation);

    setControl(
        robotCentricRequest
            .withSpeeds(speeds)
            .withDriveRequestType(DriveRequestType.Velocity)
            .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
            .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons()));
  }
  ;

  @Override
  public Command driveToRobotPose(Supplier<Pose2d> pose) {
    return runOnce(
            () -> {
              xPoseController.reset();
              yPoseController.reset();
              thetaController.reset();
            })
        .andThen(
            run(
                () -> {
                  var targetSpeeds =
                      ChassisSpeeds.discretize(
                          xPoseController.calculate(getPose().getX(), pose.get().getX())
                              * DrivetrainConstants.kMaxLinearVelocity.in(MetersPerSecond),
                          yPoseController.calculate(getPose().getY(), pose.get().getY())
                              * DrivetrainConstants.kMaxLinearVelocity.in(MetersPerSecond),
                          thetaController.calculate(
                                  getPose().getRotation().getRadians(),
                                  pose.get().getRotation().getRadians())
                              * DrivetrainConstants.kMaxAngularVelocity.in(RadiansPerSecond),
                          DrivetrainConstants.kLoopDt.in(Seconds));

                  driveRobotCentric(
                      targetSpeeds.vxMetersPerSecond,
                      targetSpeeds.vyMetersPerSecond,
                      targetSpeeds.omegaRadiansPerSecond,
                      DriveFeedforwards.zeros(4));
                }));
  }

  @Override
  public void driveToFieldPose(Pose2d pose) {
    var targetSpeeds =
        ChassisSpeeds.discretize(
            xPoseController.calculate(getPose().getX(), pose.getX())
                * DrivetrainConstants.kMaxLinearVelocity.in(MetersPerSecond),
            yPoseController.calculate(getPose().getY(), pose.getY())
                * DrivetrainConstants.kMaxLinearVelocity.in(MetersPerSecond),
            thetaController.calculate(
                    getPose().getRotation().getRadians(), pose.getRotation().getRadians())
                * DrivetrainConstants.kMaxAngularVelocity.in(RadiansPerSecond),
            DrivetrainConstants.kLoopDt.in(Seconds));

    setControl(
        fieldCentricRequest
            .withDriveRequestType(DriveRequestType.Velocity)
            .withVelocityX(targetSpeeds.vxMetersPerSecond)
            .withVelocityY(targetSpeeds.vyMetersPerSecond)
            .withRotationalRate(targetSpeeds.omegaRadiansPerSecond));
  }

  // drive with absolute heading control
  @Override
  public void driveFixedHeading(double translationX, double translationY, Rotation2d rotation) {
    var speeds =
        ChassisSpeeds.discretize(
            translationX, translationY, 0, DrivetrainConstants.kLoopDt.in(Seconds));

    setControl(
        fieldCentricFacingAngleRequest
            .withDriveRequestType(DriveRequestType.Velocity)
            .withVelocityX(speeds.vxMetersPerSecond)
            .withVelocityY(speeds.vyMetersPerSecond)
            .withTargetDirection(rotation)
            .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance));
  }

  @Override
  public void setAlignmentSetpoint(Pose2d setpoint) {
    alignmentSetpoint = setpoint;
  }

  @Override
  public boolean atPoseSetpoint() {
    return getPose().equals(alignmentSetpoint);
  }

  @Override
  public void setSwerveModuleStates(SwerveModuleState[] states) {
    for (int i = 0; i < super.getModules().length; i++) {
      super.getModule(i).apply(new ModuleRequest().withState(states[i]));
    }
  }

  @Logged(name = "MeasuredModuleStates")
  @Override
  public SwerveModuleState[] getMeasuredModuleStates() {
    return super.getState().ModuleStates;
  }

  @Logged(name = "MeasuredModulePositions")
  @Override
  public SwerveModulePosition[] getModulePositions() {
    return super.getState().ModulePositions;
  }

  @Logged(name = "TargetModuleStates")
  @Override
  public SwerveModuleState[] getTargetModuleStates() {
    return super.getState().ModuleTargets;
  }

  @Logged(name = "MeasuredRobotPose")
  @Override
  public Pose2d getPose() {
    return super.getState().Pose;
  }

  @Logged(name = "MeasuredRobotRelativeChassisSpeeds")
  @Override
  public ChassisSpeeds getChassisSpeeds() {
    return super.getState().Speeds;
  }

  @Logged(name = "MeasuredHeadingRad")
  @Override
  public Rotation2d getHeading() {
    return new Rotation2d(super.getPigeon2().getYaw().getValue().in(Radians));
  }
}
