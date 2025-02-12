/* (C) Robolancers 2025 */
package frc.robot.util;

import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;

public class FileUtil {
  public static File getDeployedFile(String name) {
    return new File(Filesystem.getDeployDirectory(), name);
  }
}
