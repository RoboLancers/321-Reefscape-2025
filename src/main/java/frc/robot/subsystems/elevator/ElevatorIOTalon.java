/* (C) Robolancers 2025 */
package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volt;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;

@Logged
// For when Elevator is real
public class ElevatorIOTalon implements ElevatorIO {
  // Creates config record w/ values
  public static final ElevatorConfig config = new ElevatorConfig(50, 0, 0, 0.3, 0.1);

  // Creates motor objects

  public TalonFX elevatorMotorLeft = new TalonFX(ElevatorConstants.kLeftMotorID);

  // NOTE: Right motor MAY be commented out in order to test one motor

  public TalonFX elevatorMotorRight = new TalonFX(ElevatorConstants.kRightMotorID);

  // Constructor: Sets up motors
  public ElevatorIOTalon() {
    setupMotors();
  }

  // Updates inputs with values from encoder (Called periodically in periodic function later)
  public void updateInputs(ElevatorInputs inputs) {
    inputs.height =
        Meters.of(
            elevatorMotorLeft.getPosition().getValueAsDouble()
                * ElevatorConstants.kElevatorConversion.in(Meters));
    inputs.velocity =
        MetersPerSecond.of(
            elevatorMotorLeft.getVelocity().getValueAsDouble()
                * ElevatorConstants.kElevatorConversion.in(Meters));
    inputs.current = Amps.of(elevatorMotorLeft.getStatorCurrent().getValueAsDouble());
  }

  // Method to setup L & R motor & encoders
  // NOTE: Right motor follows left & only left motor encoder is used
  private void setupMotors() {
    TalonFXConfiguration configurationLeft =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(ElevatorConstants.kCurrentLimit)
                    .withStatorCurrentLimitEnable(true))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(
                        ElevatorConstants.kLeftInverted
                            ? InvertedValue.Clockwise_Positive
                            : InvertedValue.CounterClockwise_Positive))
            .withFeedback(
                new FeedbackConfigs()
                    .withSensorToMechanismRatio(ElevatorConstants.kElevatorGearing)
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor));

    TalonFXConfiguration configurationRight =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(ElevatorConstants.kCurrentLimit)
                    .withStatorCurrentLimitEnable(true))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(
                        ElevatorConstants.kRightInverted
                            ? InvertedValue.Clockwise_Positive
                            : InvertedValue.CounterClockwise_Positive))
            .withFeedback(
                new FeedbackConfigs()
                    .withSensorToMechanismRatio(ElevatorConstants.kElevatorGearing)
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor));

    elevatorMotorLeft.getConfigurator().apply(configurationLeft);
    elevatorMotorRight.getConfigurator().apply(configurationRight);
  }

  // Sets power of motors w/voltage
  public void setVoltage(Voltage Volts) {
    elevatorMotorLeft.setVoltage(Volts.in(Volt));
    elevatorMotorRight.setControl(
        new Follower(elevatorMotorLeft.getDeviceID(), ElevatorConstants.kRightInverted));
  }

  // Sets encoder pos
  public void setEncoderPosition(Distance position) {
    elevatorMotorLeft.setPosition(
        position.in(Meters) / ElevatorConstants.kElevatorConversion.in(Meters));
    elevatorMotorRight.setPosition(
        position.in(Meters) / ElevatorConstants.kElevatorConversion.in(Meters));
  }

  // Special case where encoder pos is reset to the initial/starting height
  public void resetEncoderPosition() {
    setEncoderPosition(ElevatorConstants.kElevatorStartingHeight);
  }
}
