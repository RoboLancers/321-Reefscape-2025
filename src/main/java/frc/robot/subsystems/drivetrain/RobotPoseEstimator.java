/* (C) Robolancers 2025 */
package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.BuiltInAccelerometerSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotConstants;
import java.util.HashMap;

/*
 * PoseEstimator class that uses 1690 FOM system. Assumes standard deviation and FOM to be the conceptionally the same.
 * Alternative way to use this: use calculateRobotStdDev() in DrivetrainReal to update robotStdDev
 */
public class RobotPoseEstimator {
  private SwerveDriveKinematics kinematics;
  private SwerveModuleState[] moduleStates;
  private BuiltInAccelerometer accelerometer = new BuiltInAccelerometer();
  ;
  private BuiltInAccelerometerSim accelerometerSim = new BuiltInAccelerometerSim(accelerometer);
  ;

  // variables
  private double lastAccelX = 0;
  private double lastAccelY = 0;
  private double kCollisionThreshold = 2;

  // key - position, value - FOM
  private HashMap<Double, Double> xFOMPose = new HashMap<Double, Double>();
  private HashMap<Double, Double> yFOMPose = new HashMap<Double, Double>();

  public RobotPoseEstimator(SwerveDriveKinematics kinematics, SwerveModuleState[] modulesStates) {
    this.kinematics = kinematics;
    this.moduleStates = modulesStates;
  }

  public Matrix<N3, N1> calculateRobotStdDev() {
    // subtract 1 so the values are [0, infinity],
    double stdDevs = calculateSkiddingRatio() - 1;

    double translationStdDev = stdDevs * DrivetrainConstants.kTranslationStdDevCoeff;

    return VecBuilder.fill(translationStdDev, translationStdDev, 0);
  }

  // returns FOM of robot pose [1, INFINITY]
  public double calculateSkiddingRatio() {
    // find current angular velocity
    final double angularVelocityOmegaMeasured =
        kinematics.toChassisSpeeds(moduleStates).omegaRadiansPerSecond;

    // chassis rotational speeds
    final SwerveModuleState[] swerveStatesRotationalPart =
        kinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, angularVelocityOmegaMeasured));

    // chassis translational speeds
    final double[] swerveStatesTranslationalPartMagnitudes = new double[moduleStates.length];

    // for every module, convert swerve states to velocity vectors
    for (int i = 0; i < moduleStates.length; i++) {
      final Translation2d
          swerveStateMeasuredAsVector =
              convertSwerveStateToVelocityVector(moduleStates[i]), // total chassis velocity
          swerveStatesRotationalPartAsVector = // rotational velocity
          convertSwerveStateToVelocityVector(swerveStatesRotationalPart[i]),
          swerveStatesTranslationalPartAsVector = // translational velocity
          swerveStateMeasuredAsVector.minus(swerveStatesRotationalPartAsVector);
      swerveStatesTranslationalPartMagnitudes[i] = swerveStatesTranslationalPartAsVector.getNorm();
    }

    // find maximum & minimum translation
    double maximumTranslationalSpeed = 0, minimumTranslationalSpeed = Double.POSITIVE_INFINITY;
    for (double translationalSpeed : swerveStatesTranslationalPartMagnitudes) {
      maximumTranslationalSpeed = Math.max(maximumTranslationalSpeed, translationalSpeed);
      minimumTranslationalSpeed = Math.min(minimumTranslationalSpeed, translationalSpeed);
    }

    // skid ratio
    SmartDashboard.putNumber("skid ratio", maximumTranslationalSpeed / minimumTranslationalSpeed);
    return maximumTranslationalSpeed / minimumTranslationalSpeed;
  }

  private static Translation2d convertSwerveStateToVelocityVector(
      SwerveModuleState swerveModuleState) {
    return new Translation2d(swerveModuleState.speedMetersPerSecond, swerveModuleState.angle);
  }

  // TODO accelatormator doesnt give data? fix this
  public boolean detectCollision() {
    // find accelermator spike x
    double currentAccelX = RobotBase.isReal() ? accelerometer.getX() : accelerometerSim.getX();
    double jerkX = (currentAccelX - lastAccelX) / RobotConstants.kRobotLoopPeriod.in(Seconds);
    lastAccelX = currentAccelX;

    // find accelermator spike y
    double currentAccelY = RobotBase.isReal() ? accelerometer.getY() : accelerometerSim.getY();
    double jerkY = (currentAccelY - lastAccelY) / RobotConstants.kRobotLoopPeriod.in(Seconds);
    lastAccelY = currentAccelY;

    // if we get a spike >2 G then we have a collision
    return Math.abs(jerkX) > kCollisionThreshold && Math.abs(jerkY) > kCollisionThreshold;
  }

  // called periodically to update pose estimator
  public void update(SwerveModuleState[] moduleStates) {
    this.moduleStates = moduleStates;
  }

  public Pose2d getWeightedRobotPose(
      Pose2d robotPose, Pose2d visionPose, Matrix<N3, N1> visionStdDev) {
    // if we are in a collision, ignore robot pose or increase FOM
    double robotFOM = detectCollision() ? Double.POSITIVE_INFINITY : calculateSkiddingRatio();

    // TODO calculate this based on how jittery the response is
    double visionFOM = visionStdDev.get(0, 0) + 1;

    // fill maps w/ fom and poses
    xFOMPose.put(robotPose.getX(), robotFOM);
    xFOMPose.put(visionPose.getX(), visionFOM);
    yFOMPose.put(robotPose.getY(), robotFOM);
    yFOMPose.put(visionPose.getY(), visionFOM);

    // calculate weighted avg of translational pose
    double weightedPoseX = calculateWeightedAverage(xFOMPose);
    double weightedPoseY = calculateWeightedAverage(yFOMPose);

    // clear maps for next iteration
    xFOMPose.clear();
    yFOMPose.clear();

    return new Pose2d(weightedPoseX, weightedPoseY, robotPose.getRotation());
  }

  // calculate weighted average based on 1690's pose merging
  private double calculateWeightedAverage(HashMap<Double, Double> map) {
    double num = 0;
    double denom = 0;

    for (HashMap.Entry<Double, Double> entry : map.entrySet()) {
      num += (1 / Math.pow(entry.getValue(), 2)) * entry.getKey();
      denom += (1 / Math.pow(entry.getValue(), 2));
    }

    return num / denom;
  }
}
