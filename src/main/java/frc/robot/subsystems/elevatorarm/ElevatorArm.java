/* (C) Robolancers 2025 */
package frc.robot.subsystems.elevatorarm;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.TunableConstant;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AnalogSensorConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

// NOTE: Convention is: zeroed when coral intake CG is 90 degrees
// TO ZERO: move coral intake such that CG is all the way down due to gravity, zero, then move it to
// 90 deg, then
// zero again

// Elevator Arm subsystem - represents the arm/pivot on the elevator
@Logged
public class ElevatorArm extends SubsystemBase {
  
  // controllers classes for controlling the arm
  private PIDController pidController = new PIDController(0.25, 0, 0.002);
  private ArmFeedforward feedforward = new ArmFeedforward(0, 0.330, 0,0);

  private SparkMax armMotor =
     new SparkMax(ElevatorArmConstants.kElevatorArmId, MotorType.kBrushless);

  // suppliers for game piece detection for variable feedforward for game pieces
  @NotLogged private BooleanSupplier hasCoral = () -> false;

  /**
   * Creates an ElevatorArm instance controlling a real or simulated elevator arm based on the
   * environment
   */
  public static ElevatorArm create() {
   return new ElevatorArm();
  }

  public ElevatorArm() {
    this.pidController.setTolerance(ElevatorArmConstants.kAngleTolerance.in(Degrees));
    configureMotors();
  }

  public void configureMotors(){
    armMotor.configure(
        new SparkMaxConfig() // config for basic motor stuff
            .smartCurrentLimit(ElevatorArmConstants.kCurrentLimit)
            .voltageCompensation(ElevatorArmConstants.kNominalVoltage)
            .idleMode(IdleMode.kBrake)
            .inverted(ElevatorArmConstants.kInverted)
            .apply(
                new EncoderConfig() // config for relative encoder
                    .positionConversionFactor(ElevatorArmConstants.kPositionConversionFactor)
                    .velocityConversionFactor(ElevatorArmConstants.kVelocityConversionFactor))
            .apply(new AnalogSensorConfig().positionConversionFactor(360 / 3.3)),
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

 

  /**
   * Commands the arm to go to a desired angle This needs to be run continuously (see {@link
   * ElevatorArm#goToAngle(Supplier<Angle>) goToAngle})
   *
   * @param angle the angle to command the arm to go to
   */
  public void goToAngle(Angle angle) {

    double volts =
        pidController.calculate(getAngle().in(Degrees), angle.in(Degrees))
            + feedforward.calculate(
                angle.plus(ElevatorArmConstants.kCMOffset).in(Radians),
                0);

    runVolts(Volts.of(volts));
  }

  /**
   * Commands the arm to run at a certain voltage This needs to be run continuously (see {@link
   * ElevatorArm#runVolts(Supplier<Voltage>) runVolts})
   *
   * @param volts the voltage to run the arm at
   */
  public void runVolts(Voltage volts) {
    armMotor.setVoltage(volts);
  }


  public Command goToAnglePID(Supplier<Angle> angleSup) {
    return run(() -> {
          goToAngle(angleSup.get());
        });
  }

  /**
   * Creates a command that runs the arm at a certain voltage
   *
   * @param volts A supplier that gives the voltage for the arm to run at
   * @return a command that runs the arm at the voltage suppplied by the Supplier<Voltage>
   */
  public Command runVolts(Supplier<Voltage> volts) {
    return run(
        () -> {
          runVolts(volts.get());
        });
  }


  /**
   * Creates a Command that allows the user to tune the ElevatorArm using SmartDashboard
   *
   * <p>Parameters: kP, kI, kD, kG, TargetAngle
   *
   * @return the command used to tune
   */
  public Command tune() {
    TunableConstant kP = new TunableConstant("/ElevatorArm/kP",0);
    TunableConstant kI = new TunableConstant("/ElevatorArm/kI", 0);
    TunableConstant kD = new TunableConstant("/ElevatorArm/kD", 0);
    TunableConstant kG = new TunableConstant("/ElevatorArm/kG", 0);
    TunableConstant targetAngle = new TunableConstant("/ElevatorArm/TargetAngle", 0);

    return run(
        () -> {
          this.pidController.setPID(kP.get(), kI.get(), kD.get());
          this.feedforward = new ArmFeedforward(0, kG.get(), 0);
          goToAngle(Degrees.of(targetAngle.get()));
        });
  }

  
  public boolean atSetpoint() {
    return pidController.atSetpoint();
  }

  public Angle getAngle() {
  Angle angle = Degrees.of(armMotor.getAnalog().getPosition()); 
    return angle;
  }

  public AngularVelocity getVelocity(){
   AngularVelocity velocity = DegreesPerSecond.of(armMotor.getEncoder().getVelocity());
   return velocity;
  }

  public Current getCurrent(){
    Current current = Amps.of(armMotor.getOutputCurrent());
    return current;
  }

  public boolean atAngle(Angle angle) {
    return Math.abs(getAngle().in(Degrees) - angle.in(Degrees))
        < ElevatorArmConstants.kAngleTolerance.in(Degrees);
  }
}
