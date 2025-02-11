/* (C) Robolancers 2025 */
package frc.robot.subsystems.algaeIntakeRollers;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;

// Spark implementation of real mechanism
@Logged
public class AlgaeIntakeRollersIOSpark implements AlgaeIntakeRollersIO {

  // placeholder device ids
  public static final AlgaeIntakeRollersConfig config = new AlgaeIntakeRollersConfig(0, 0, 0, 0);
  private DigitalInput algaeSensor = new DigitalInput(AlgaeIntakeRollersConstants.kBeamBreakId);

  private SparkMax rollerMotorLeft =
      new SparkMax(AlgaeIntakeRollersConstants.kMotorId, MotorType.kBrushless);

  public AlgaeIntakeRollersIOSpark() {
    configureMotors(); // configures motors once algae spark object is created
  }

  public void configureMotors() {
    rollerMotorLeft.configure( // configures single motor
        new SparkMaxConfig()
            .inverted(AlgaeIntakeRollersConstants.kRollerInverted)
            .voltageCompensation(AlgaeIntakeRollersConstants.kNominalVoltage.in(Volts))
            .smartCurrentLimit(AlgaeIntakeRollersConstants.kSmartCurrentLimit),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  public void setRollerVoltage(Voltage volts) {
    rollerMotorLeft.setVoltage(volts); // sets sim voltage
  }

  public void updateInputs(AlgaeIntakeRollersInputs inputs) {
    inputs.hasAlgae = algaeSensor.get(); // gets info for updating inputs
    inputs.rollerVelocity = RPM.of(rollerMotorLeft.getEncoder().getVelocity());
  }
}
