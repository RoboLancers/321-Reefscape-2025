/* (C) Robolancers 2025 */
package frc.robot.subsystems.algaeIntakePivot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;

@Logged
public class AlgaeIntakePivotIOKraken implements AlgaeIntakePivotIO {
  // kraken implementation of real mechanism
  public static final AlgaeIntakePivotConfig config = new AlgaeIntakePivotConfig(0, 0, 0, 0);

  private TalonFX pivotMotorLeft =
      new TalonFX(
          AlgaeIntakePivotConstants.kPivotMotorLeftId); // corresponds to the left pivot motor
  private TalonFX pivotMotorRight =
      new TalonFX(
          AlgaeIntakePivotConstants.kPivotMotorRightId); // corresponds to the right pivot motor

  private VoltageOut voltageRequest = new VoltageOut(0); // used to set voltage
  private Follower followRequest =
      new Follower(
          AlgaeIntakePivotConstants.kPivotMotorLeftId, AlgaeIntakePivotConstants.kRightInverted);

  public AlgaeIntakePivotIOKraken() {
    pivotMotorLeft // sets up and creates left pivot motor
        .getConfigurator()
        .apply(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(AlgaeIntakePivotConstants.kSmartCurrentLimit));
    pivotMotorLeft
        .getConfigurator()
        .apply(
            new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(
                    AlgaeIntakePivotConstants.kLeftInverted
                        ? InvertedValue.CounterClockwise_Positive
                        : InvertedValue.Clockwise_Positive));
    pivotMotorLeft
        .getConfigurator()
        .apply(
            new FeedbackConfigs()
                .withSensorToMechanismRatio(AlgaeIntakePivotConstants.kPivotGearing));

    pivotMotorRight // same thing with right pivot motor
        .getConfigurator()
        .apply(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(AlgaeIntakePivotConstants.kSmartCurrentLimit));
    pivotMotorRight
        .getConfigurator()
        .apply(
            new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(
                    AlgaeIntakePivotConstants.kRightInverted
                        ? InvertedValue.CounterClockwise_Positive
                        : InvertedValue.Clockwise_Positive));
    pivotMotorRight
        .getConfigurator()
        .apply(
            new FeedbackConfigs()
                .withSensorToMechanismRatio(AlgaeIntakePivotConstants.kPivotGearing));
  }

  public void setPivotVoltage(Voltage volts) {
    pivotMotorLeft.setControl(
        voltageRequest.withOutput(volts)); // kraken implementation of setVoltage
    pivotMotorRight.setControl(followRequest);
  }

  public void updateInputs(AlgaeIntakePivotInputs inputs) {
    // updates inputs
    inputs.pivotAngle = pivotMotorLeft.getPosition().getValue();
    inputs.pivotVelocity = pivotMotorLeft.getVelocity().getValue();
    inputs.pivotCurrent = pivotMotorLeft.getStatorCurrent().getValue();
  }

  public void resetEncoder(Angle angle) {
    pivotMotorLeft.setPosition(angle);
  }
}
