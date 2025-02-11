/* (C) Robolancers 2025 */
package frc.robot.subsystems.coralendeffector;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Voltage;

// Updates inputs and sets voltage
@Logged
public interface CoralEndEffectorIO {
  default void updateInputs(CoralEndEffectorInputs inputs) {}

  default void setVoltage(Voltage voltage) {}
}
