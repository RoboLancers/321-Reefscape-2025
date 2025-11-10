/* (C) Robolancers 2025 */
package frc.robot.subsystems.coralendeffector;

import static edu.wpi.first.units.Units.Millimeter;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.TunableConstant;
import java.util.function.Supplier;

import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

// coral end effector subsystem (now a coral / algae end effector mechanism)
@Logged
public class CoralEndEffector extends SubsystemBase {
  
  private PIDController endEffectorController = new PIDController(0.00007, 0, 0);
  private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0,0.0022,0);

  private SparkMax motor = new SparkMax(CoralEndEffectorConstants.kMotorPort, MotorType.kBrushless);
  
  private void configureMotors(){
  motor.configure(
      new SparkMaxConfig()
          .inverted(CoralEndEffectorConstants.kInvertedMotor)
          .smartCurrentLimit(CoralEndEffectorConstants.kCurrentLimit),
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);
  }

  // init distance sensor
  private TimeOfFlight coralDistSensor = new TimeOfFlight(CoralEndEffectorConstants.kCoralSensorId);

  public static CoralEndEffector create() {
    return new CoralEndEffector();
  }

  public CoralEndEffector() {
    configureMotors();
  }

  // run the end effector at a certain specified velocity using PIDFF control
  // Will only run once; For a continuous method, see runAtVelocity(Supplier<AngularVelocity>)
  public void runAtVelocity(AngularVelocity velocity) {
    double output =
        endEffectorController.calculate(getVelocity().in(RPM), velocity.in(RPM))
            + feedforward.calculate(velocity.in(RPM));
    motor.setVoltage(Volts.of(output));
  }

  // continuously run the end effector at a certain velocity supplied by the velocity supplier
  public Command runAtVelocity(Supplier<AngularVelocity> velocity) {
    return run(
        () -> {
          runAtVelocity(velocity.get());
        });
  }

  public Command runVolts(Supplier<Voltage> voltage) {
    return run(() -> motor.setVoltage(voltage.get()));
  }

  public boolean hasCoral() {
   return coralDistSensor.getRange() < CoralEndEffectorConstants.kDetectionRange.in(Millimeter);
  }

  // stalls coral if we have a coral; this should be the default command
  public Command stallCoralIfDetected() {
    return runAtVelocity(
        () -> {
          if (hasCoral()) {
            return CoralEndEffectorConstants.kCoralStallRPM;
          }
          return RPM.of(0);
        });
  }

  // tune PIDFF of end effector
  public Command tune() {
    TunableConstant kP = new TunableConstant("/CoralEndEffector/kP", 0.0007);
    TunableConstant kI = new TunableConstant("/CoralEndEffector/kI", 0);
    TunableConstant kD = new TunableConstant("/CoralEndEffector/kD", 0);
    TunableConstant kV = new TunableConstant("/CoralEndEffector/kV", 0.0022);
    TunableConstant targetRPM = new TunableConstant("/CoralEndEffector/TargetRPM", 0);

    return run(
        () -> {
          endEffectorController.setPID(kP.get(), kI.get(), kD.get());
          feedforward = new SimpleMotorFeedforward(0, kV.get());
          runAtVelocity(RPM.of(targetRPM.get()));
        });
  }

 private AngularVelocity getVelocity(){
  return RPM.of(this.motor.getEncoder().getVelocity());
 }

}
