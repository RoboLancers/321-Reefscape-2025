/* (C) Robolancers 2025 */
package frc.robot.subsystems.algaeIntakeRollers;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Voltage;

@Logged
public class AlgaeIntakeRollersConstants {

  // motor, sensor IDs
  public static final int kBeamBreakId = 0;
  public static final int kMotorId = 0;

  // function constants
  public static final Voltage kRollerIntakeVoltage = Volts.of(2); // TODO: tune
  public static final Voltage kRollerOuttakeVoltage = Volts.of(-2); // TODO: tune
  public static final Voltage kStallVoltage = Volts.of(1); // TODO: tune

  // physical constants
  public static final double kRollerGearing = 1; // TODO: find
  public static final double kRollerMOI = 0.01; // TODO: find

  // motor configurations
  public static final boolean kRollerInverted = false;
  public static final int kSmartCurrentLimit = 40;
  public static final double kRollerPositionConversionFactor = 1;
  public static final double kRollerVelocityConversionFactor = 1;
  public static final Voltage kNominalVoltage = Volts.of(12);
}
