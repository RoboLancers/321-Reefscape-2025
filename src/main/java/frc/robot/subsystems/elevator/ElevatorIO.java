/* (C) Robolancers 2025 */
package frc.robot.subsystems.elevator;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;

@Logged
public interface ElevatorIO {

  default void updateInputs(ElevatorInputs inputs) {}

  default void setVoltage(Voltage volts) {}

  default void setEncoderPosition(Distance position) {}
}
