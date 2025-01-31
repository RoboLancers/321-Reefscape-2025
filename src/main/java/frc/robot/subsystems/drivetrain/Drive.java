/* (C) Robolancers 2025 */
package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.MathUtils;
import java.util.function.DoubleSupplier;

@Logged

/*
 * Drivetrain based on CTR SwerveDrive
 */

public class Drive extends SubsystemBase {
  // auto gains
  public record Gains(double kP, double kI, double kD) {}

  public static final Gains kPPDrive = new Gains(5, 0, 0);
  public static final Gains kPPTurn = new Gains(5, 0, 0);

  DriveIO io;
  DriveInputs inputs;

  public static Drive create() {
    return RobotBase.isReal()
        ? new Drive(
            new DriveIOKraken(
                TunerConstants.kTunerDrivetrain.getDriveTrainConstants(),
                TunerConstants.kTunerDrivetrain.getModuleConstants()))
        : new Drive(
            new DriveIOSim(
                TunerConstants.kTunerDrivetrain.getDriveTrainConstants(),
                TunerConstants.kTunerDrivetrain.getModuleConstants()));
  }

  public static Drive disable() {
    return new Drive(new DriveIOIdeal());
  }

  public Drive(DriveIO io) {
    this.io = io;
    inputs = new DriveInputs();
  }

  public void configureAutoBuilder() {
    try { // try and catch for config exception
      var config = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
          () -> inputs.pose, // Supplier of current robot pose
          (pose) -> io.resetPose(pose), // Consumer for seeding pose against auto
          () -> inputs.chassisSpeeds, // Supplier of current robot speeds
          RobotBase.isReal()
              ? (speeds, feedforwards) -> // real drive chassisSpeeds consumer
              io.setControl(
                      new SwerveRequest.ApplyRobotSpeeds()
                          .withSpeeds(speeds)
                          .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                          .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons()))
              : (speeds, feedforwards) ->
                  io.setSimControl(speeds), // sim drive chassisSpeeds consumer
          new PPHolonomicDriveController(
              // PID constants for translation
              new PIDConstants(kPPDrive.kP(), kPPDrive.kI(), kPPDrive.kD()),
              // PID constants for rotation
              new PIDConstants(kPPTurn.kP(), kPPTurn.kI(), kPPTurn.kD())),
          config,
          // Assume the path needs to be flipped for Red vs Blue, this is normally the case
          () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
          this // Subsystem for requirements
          );
    } catch (Exception ex) {
      DriverStation.reportError(
          "Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
    }
  }

  // real drive teleop command
  public Command driveFieldCentric(
      DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation) {

    // apply deadband first, then square rotation, then convert to m/s
    double newTranslationX =
        MathUtil.applyDeadband(translationX.getAsDouble(), DriveConstants.kDriveDeadband)
            * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    double newTranslationY =
        MathUtil.applyDeadband(translationY.getAsDouble(), DriveConstants.kDriveDeadband)
            * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    double newRotation =
        MathUtils.squareKeepSign(
                MathUtil.applyDeadband(rotation.getAsDouble(), DriveConstants.kRotationDeadband))
            * DriveConstants.kMaxAngularVelocity;

    // fix swerve drift
    var speeds =
        ChassisSpeeds.discretize(
            newTranslationX, newTranslationY, newRotation, DriveConstants.kLoopDt.magnitude());

    // x braking
    // if(Math.abs(newTranslationX) < DriveConstants.kDriveDeadband &&
    // Math.abs(newTranslationY) < DriveConstants.kDriveDeadband &&
    // Math.abs(newRotation) < DriveConstants.kRotationDeadband){

    // return run (
    //     () -> {
    //       io.setControl(new SwerveRequest.SwerveDriveBrake());
    //     }
    //   );
    // }

    return run(
        () -> {
          io.setControl(
              new SwerveRequest.FieldCentric()
                  .withDriveRequestType(DriveRequestType.Velocity)
                  .withVelocityX(speeds.vxMetersPerSecond)
                  .withVelocityY(speeds.vyMetersPerSecond)
                  .withRotationalRate(speeds.omegaRadiansPerSecond));
        });
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }
}
