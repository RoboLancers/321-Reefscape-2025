/* (C) Robolancers 2025 */
package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;

@Logged
// For when Elevator is real
public class ElevatorIOSpark implements ElevatorIO {
  // Creates config record with values
  public static final ElevatorConfig config = new ElevatorConfig(150, 0, 0.3, 0.43, 0.64);
  // Creates motor objects
  public SparkMax elevatorMotorLeft =
      new SparkMax(ElevatorConstants.kLeftMotorID, MotorType.kBrushless);

  public SparkMax elevatorMotorRight =
      new SparkMax(ElevatorConstants.kRightMotorID, MotorType.kBrushless);

  // Constructor: Sets up motors
  public ElevatorIOSpark() {
    setupMotors();
  }

  // Updates inputs with values from encoder (Called periodically in periodic function later)
  public void updateInputs(ElevatorInputs inputs) {
    inputs.height = Meters.of(elevatorMotorLeft.getEncoder().getPosition());
    inputs.velocity = MetersPerSecond.of(elevatorMotorLeft.getEncoder().getVelocity());
    inputs.current = Amps.of(elevatorMotorLeft.getOutputCurrent());
  }

  // Method to setup L & R motor & encoders
  // NOTE: Right motor follows left & only left motor encoder is used
  private void setupMotors() {
    elevatorMotorLeft.configure(
        new SparkMaxConfig()
            .smartCurrentLimit(ElevatorConstants.kCurrentLimit)
            .inverted(ElevatorConstants.kLeftInverted)
            .apply(
                new EncoderConfig()
                    .velocityConversionFactor(ElevatorConstants.kVelocityConversionFactor)
                    .positionConversionFactor(ElevatorConstants.kPositionConversionFactor)),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    elevatorMotorRight.configure(
        new SparkMaxConfig()
            .smartCurrentLimit(ElevatorConstants.kCurrentLimit)
            .apply(
                new EncoderConfig()
                    .velocityConversionFactor(ElevatorConstants.kVelocityConversionFactor)
                    .positionConversionFactor(ElevatorConstants.kPositionConversionFactor))
            .follow(elevatorMotorLeft, ElevatorConstants.kRightInverted),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  // Sets power of motors w/voltage
  public void setVoltage(Voltage Volts) {
    elevatorMotorLeft.setVoltage(Volts);
  }

  // Sets encoder pos
  public void setEncoderPosition(Distance position) {
    elevatorMotorLeft.getEncoder().setPosition(position.in(Meters));
  }

  // Special case where encoder pos is reset to the initial/starting height
  public void resetEncoderPosition() {
    setEncoderPosition(ElevatorConstants.kElevatorStartingHeight);
  }
}
