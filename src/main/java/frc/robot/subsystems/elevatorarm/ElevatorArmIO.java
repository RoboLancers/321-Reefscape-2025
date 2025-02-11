/* (C) Robolancers 2025 */
package frc.robot.subsystems.elevatorarm;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Voltage;

// Interface for facilitating hardware abstraction
@Logged
public interface ElevatorArmIO {
  // updates the inputs provided in the parameters from sensors
  default void updateInputs(ElevatorArmInputs inputs) {}

  // sets voltage to the arm based on the provided voltage
  default void setVoltage(Voltage volts) {}
}
