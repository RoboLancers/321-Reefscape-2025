/* (C) Robolancers 2025 */
package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.util.FileUtil;
import java.nio.file.Path;

public class VisionConstants {
  public static record CameraConfig(
      String cameraName,
      int resolutionWidth,
      int resolutionHeight,
      Path jsonConfigPath,
      Transform3d robotToCamera) {}

  // TODO: fill out CameraConfig & get config jsons for all cameras

  public static final CameraConfig kUSBCameraConfig =
      new CameraConfig(
          "USB_Camera",
          1280,
          800,
          FileUtil.getDeployedFile("").toPath(), // TODO: get config from photon client
          new Transform3d(
              Meters.of(-0.27),
              Meters.zero(),
              Meters.zero(),
              new Rotation3d(Degrees.zero(), Degrees.of(30), Degrees.of(180))));

  public static final CameraConfig[] k427CameraConfigs = {kUSBCameraConfig};
  public static final CameraConfig[] k321CameraConfigs = {};

  public static final double kTranslationStdDevCoeff = 1.0;
  public static final double kRotationStdDevCoeff = 1.0;

  public static final AprilTagFieldLayout kAprilTagFieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
}
