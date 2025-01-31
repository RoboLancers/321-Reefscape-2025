/* (C) Robolancers 2025 */
package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

@Logged
public class DriveIOSim extends DriveIOKraken {

  private final SelfControlledSwerveDriveSimulation simulatedDrive;
  private final Field2d field2d;

  public DriveIOSim(
      SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
    // super constructor does nothing
    super(drivetrainConstants, modules);

    final DriveTrainSimulationConfig config =
        DriveTrainSimulationConfig.Default()
            .withGyro(COTS.ofPigeon2())
            .withSwerveModule(
                COTS.ofMark4(
                    DCMotor.getKrakenX60(1),
                    DCMotor.getKrakenX60(1),
                    COTS.WHEELS.COLSONS.cof, // Use the COF for Colson Wheels
                    3)) // L3 Gear ratio
            // Configures the track length and track width (spacing between swerve modules)
            .withTrackLengthTrackWidth(Inches.of(29), Inches.of(29))
            // Configures the bumper size (dimensions of the robot bumper) trackwidth + 6 inches
            .withBumperSize(Inches.of(35), Inches.of(35));

    this.simulatedDrive =
        new SelfControlledSwerveDriveSimulation(
            new SwerveDriveSimulation(config, new Pose2d(3, 3, new Rotation2d())));

    // Register the drivetrain simulation to the simulation world
    SimulatedArena.getInstance().addDriveTrainSimulation(simulatedDrive.getDriveTrainSimulation());

    // A field2d widget for debugging
    field2d = new Field2d();
    SmartDashboard.putData("simulation field", field2d);
  }

  @Override
  public void resetPose(Pose2d pose) {
    simulatedDrive.resetOdometry(pose);
  }

  @Override
  public void addVisionMeasurement(Pose2d visionEstimate, double timestamp) {
    simulatedDrive.addVisionEstimation(visionEstimate, timestamp);
  }

  @Override
  public void setSimControl(ChassisSpeeds speeds) {
    simulatedDrive.runChassisSpeeds(speeds, new Translation2d(), true, true);
  }

  @Override
  public void updateInputs(DriveInputs inputs) {
    inputs.moduleStates = simulatedDrive.getMeasuredStates();
    inputs.moduleTargetStates = simulatedDrive.getSetPointsOptimized();
    inputs.chassisSpeeds = simulatedDrive.getActualSpeedsRobotRelative();
    inputs.pose = simulatedDrive.getOdometryEstimatedPose();

    inputs.yaw =
        new Rotation2d(
            simulatedDrive
                .getDriveTrainSimulation()
                .getGyroSimulation()
                .getGyroReading()
                .getRadians());
    inputs.pitch = new Rotation2d();
    inputs.roll = new Rotation2d();

    // add simulation periodic to updateInputs, so it loops in drive
    simulatedDrive.periodic();
    this.updateSimState(0.02, RobotController.getBatteryVoltage());

    // send simulation data to dashboard for testing
    field2d.setRobotPose(simulatedDrive.getActualPoseInSimulationWorld());
    field2d.getObject("odometry").setPose(simulatedDrive.getOdometryEstimatedPose());
  }
}
