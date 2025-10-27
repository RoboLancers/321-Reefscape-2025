/* (C) Robolancers 2025 */
package frc.robot.subsystems.elevatorarm;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
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
  private TrapezoidProfile profile;
  private TrapezoidProfile.State setpointState = new TrapezoidProfile.State();
  private double goalAngle;
  private boolean hasSeeded = false;

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
    this.profile = new TrapezoidProfile(ElevatorArmConstants.kArmConstraints);
    configureMotors();
    seedEncoderValues();
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
   * Calculates the amount of feedforward needed to keep the arm with the game piece up
   *
   * @return the amount of feedforward (in volts) to keep the arm with game piece up
   */
  private double calculateGamepieceFeedforward(Angle targetAngle) {
    // calculate the amount of feedforward needed to keep a coral
    double output = 0;
    if (hasCoral.getAsBoolean()) {
      output += ElevatorArmConstants.kCoralFF * Math.cos(targetAngle.in(Radians));
    }
    return output;
  }


  /**
   * Commands the arm to go to a desired angle This needs to be run continuously (see {@link
   * ElevatorArm#goToAngle(Supplier<Angle>) goToAngle})
   *
   * @param angle the angle to command the arm to go to
   */
  public void goToAngle(Angle angle) {
    setpointState =
        profile.calculate(
            RobotConstants.kRobotLoopPeriod.in(Seconds),
            setpointState,
            new TrapezoidProfile.State(angle.in(Degrees), 0));

    double volts =
        pidController.calculate(getAngle().in(Degrees), setpointState.position)
            + feedforward.calculate(
                Degrees.of(setpointState.position).plus(ElevatorArmConstants.kCMOffset).in(Radians),
                0)
            + calculateGamepieceFeedforward(Degrees.of(setpointState.position));

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

  /**
   * Creates a command that runs the arm to a certain angle NOTE: this command NEVER ends
   *
   * @param angleSup A supplier that supplies the angle for the arm to go to
   * @return a command that runs the arm to the desired angle supplied by the Supplier<Angle>
   */
  public Command goToAngleProfiled(Supplier<Angle> angleSup) {
    return run(() -> {
          goalAngle = angleSup.get().in(Degrees);
          goToAngle(angleSup.get());
        })
        .beforeStarting(() -> profile = new TrapezoidProfile(ElevatorArmConstants.kArmConstraints));
  }

  public Command goToAnglePID(Supplier<Angle> angleSup) {
    return run(() -> {
          goalAngle = angleSup.get().in(Degrees);
          goToAngle(angleSup.get());
        })
        .beforeStarting(() -> profile = new TrapezoidProfile(new Constraints(0, 0)));
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

  public void seedEncoderValues() {
    armMotor.getEncoder().setPosition(armMotor.getAnalog().getPosition() - 180);
  }

  public Command seedEncoder() {
    return Commands.runOnce(
        () -> {
          seedEncoderValues();
          System.out.println("Seeded.");
          this.hasSeeded = true;
        });
  }

  public boolean hasSeeded() {
    return hasSeeded;
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
    TunableConstant maxVelocity = new TunableConstant("/ElevatorArm/MaxVelocity", 0);
    TunableConstant maxAcceleration = new TunableConstant("/ElevatorArm/MaxAcceleration", 0);

    return run(
        () -> {
          this.pidController.setPID(kP.get(), kI.get(), kD.get());
          this.feedforward = new ArmFeedforward(0, kG.get(), 0);
          this.profile =
              new TrapezoidProfile(
                  new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get()));
          goalAngle = targetAngle.get();
          goToAngle(Degrees.of(targetAngle.get()));
        });
  }

  
  public boolean atSetpoint() {
    return pidController.atSetpoint();
  }

  public boolean atGoal() {
    return MathUtil.isNear(
        setpointState.position, goalAngle, ElevatorArmConstants.kAngleTolerance.in(Degrees));
  }

  public Angle getAngle() {
  Angle angle = Degrees.of(armMotor.getEncoder().getPosition()); 
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

  public double getSetpoint() {
    return setpointState.position;
  }

  public boolean atAngle(Angle angle) {
    return Math.abs(getAngle().in(Degrees) - angle.in(Degrees))
        < ElevatorArmConstants.kAngleTolerance.in(Degrees);
  }
}
