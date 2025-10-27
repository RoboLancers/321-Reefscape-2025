/* (C) Robolancers 2025 */
package frc.robot.subsystems.algaeIntakeRollers;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;

// spark implementation of real mechanism
@Logged
public class AlgaeIntakeRollersIOKraken implements AlgaeIntakeRollersIO {

  // device ids are placeholders
  public static final AlgaeIntakeRollersConfig config = new AlgaeIntakeRollersConfig(0, 0, 0, 0);
  private DigitalInput algaeSensor = new DigitalInput(AlgaeIntakeRollersConstants.kBeamBreakId);

  private TalonFX rollerMotor = new TalonFX(AlgaeIntakeRollersConstants.kMotorId);

  private VoltageOut voltageRequest = new VoltageOut(0);

  public AlgaeIntakeRollersIOKraken() {
    configureMotors(); // configures motors once algae kraken object is created
  }

  public void configureMotors() {
    rollerMotor
        .getConfigurator()
        .apply(
            new MotorOutputConfigs()
                .withInverted(
                    AlgaeIntakeRollersConstants.kRollerInverted
                        ? InvertedValue.Clockwise_Positive
                        : InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake));

    rollerMotor
        .getConfigurator()
        .apply(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(AlgaeIntakeRollersConstants.kSmartCurrentLimit)
                .withStatorCurrentLimitEnable(true));

    rollerMotor
        .getConfigurator()
        .apply(
            new FeedbackConfigs()
                .withSensorToMechanismRatio(AlgaeIntakeRollersConstants.kRollerGearing));
  }

  public void setRollerVoltage(Voltage volts) {
    rollerMotor.setControl(voltageRequest.withOutput(volts));
  }

  public void updateInputs(AlgaeIntakeRollersInputs inputs) {
    inputs.hasAlgae = algaeSensor.get(); // gets info for updating inputs
    inputs.rollerVelocity = rollerMotor.getVelocity().getValue();
  }
}
