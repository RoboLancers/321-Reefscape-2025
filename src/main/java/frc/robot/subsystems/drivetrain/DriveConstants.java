/* (C) Robolancers 2025 */
package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Time;

@Logged
public class DriveConstants {
  public static final double kDriveDeadband = 0.03;
  public static final double kRotationDeadband = 0.03;
  public static final double kMaxAngularVelocity = Math.PI;

  public static final Time kLoopDt = Seconds.of(0.02);
}
