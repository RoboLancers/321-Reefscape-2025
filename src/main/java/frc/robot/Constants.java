/* (C) Robolancers 2025 */
package frc.robot;

import static edu.wpi.first.units.Units.Amps;

import edu.wpi.first.units.measure.Current;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    public static final Current kDriveCurrentLimit = Amps.of(60);
  }

  public static final class RollerConstants {
    public static final int kRollerMotorID = 5;
    public static final Current kRollerCurrentLimit = Amps.of(60);
    public static final double kRollerVoltageCompensation = 10;
    public static final double kRollerEjectValue = 0.44;
  }

  public static final class OperatorConstants {
    public static final int kDriverController = 0;
    public static final int kManipulatorController = 1;
  }
}
