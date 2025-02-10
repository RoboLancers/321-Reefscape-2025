/* (C) Robolancers 2025 */
package frc.robot.subsystems.elevatorarm;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.RobotConstants;

/** Implementation of the ElevatorArmIO that controls a simulated ElevatorArm */
@Logged
public class ElevatorArmIOSim implements ElevatorArmIO {

  // tuning config for the ElevatorArmIOSim
  public static final ElevatorArmConfig config = new ElevatorArmConfig(0.13, 0, 0.004, 2.244, 0);

  // simulated instance of the elevator arm
  private SingleJointedArmSim simMotor =
      new SingleJointedArmSim(
          DCMotor.getNEO(1),
          ElevatorArmConstants.kElevatorArmGearing,
          ElevatorArmConstants.kElevatorArmMOI,
          ElevatorArmConstants.kElevatorArmLength.in(Meters),
          ElevatorArmConstants.kMinAngle.plus(ElevatorArmConstants.kCMOffset).in(Radians),
          ElevatorArmConstants.kMaxAngle.plus(ElevatorArmConstants.kCMOffset).in(Radians),
          true,
          ElevatorArmConstants.kStartAngle.in(Radians));

  // update inputs from the arm simulation
  public void updateInputs(ElevatorArmInputs inputs) {
    simMotor.update(RobotConstants.kRobotLoopPeriod.in(Seconds));
    inputs.angle = Radians.of(simMotor.getAngleRads() - ElevatorArmConstants.kCMOffset.in(Radians));
    inputs.velocity = RadiansPerSecond.of(simMotor.getVelocityRadPerSec());
    inputs.current = Amps.of(simMotor.getCurrentDrawAmps());
  }

  // set voltage to the arm simulation
  public void setVoltage(Voltage volts) {
    simMotor.setInputVoltage(volts.in(Volts));
  }
}
