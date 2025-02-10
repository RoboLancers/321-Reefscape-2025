/* (C) Robolancers 2025 */
package frc.robot.subsystems.algaeIntakeRollers;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotConstants;

@Logged
public class AlgaeIntakeRollersIOSim implements AlgaeIntakeRollersIO {

  public static final AlgaeIntakeRollersConfig config =
      new AlgaeIntakeRollersConfig(0.1, 0, 0, 0.002075);
  private DCMotorSim rollerSim;

  public AlgaeIntakeRollersIOSim() {
    rollerSim = // creates and configures a single motor sim to represent the mechanism's rollers
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getNEO(1),
                AlgaeIntakeRollersConstants.kRollerMOI,
                AlgaeIntakeRollersConstants.kRollerGearing),
            DCMotor.getNEO(1));
    SmartDashboard.putBoolean("SimInputs/AlgaeIntakeRollers/HasAlgae", false);
  }

  public void setRollerVoltage(Voltage volts) {
    rollerSim.setInputVoltage(volts.in(Volts));
  }

  public void updateInputs(AlgaeIntakeRollersInputs inputs) {
    // update information from the DCMotor sim & dashboard inputs
    rollerSim.update(RobotConstants.kRobotLoopPeriod.in(Seconds));
    inputs.rollerVelocity = RadiansPerSecond.of(rollerSim.getAngularVelocityRadPerSec());
    inputs.hasAlgae = SmartDashboard.getBoolean("SimInputs/AlgaeIntakeRollers/HasAlgae", false);
  }
}
