/* (C) Robolancers 2025 */
package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volt;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.TunableConstant;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

@Logged
public class Elevator extends SubsystemBase {
  // Initialize all parts of Elevator Subsystem, as well as pid controller
 
  private Distance targetHeight = ElevatorConstants.kElevatorStartingHeight;

  private boolean isHomed = false;

  private Distance lastTargetHeight = ElevatorConstants.kElevatorStartingHeight;

// Creates motor objects

  public TalonFX elevatorMotorLeft = new TalonFX(ElevatorConstants.kLeftMotorID);

  // NOTE: Right motor MAY be commented out in order to test one motor

  public TalonFX elevatorMotorRight = new TalonFX(ElevatorConstants.kRightMotorID);

  // Method that creates the Elevator object as the real/sim io by checking if we're running a sim
  // or not
  public static Elevator create() {
    return new Elevator();
  }

  // Eleavtor Constructor
  // Creates Elevator, sets initialized variables to the real/sim values from create() method above
  public Elevator() {
    setupMotors();
    setOnboardPID(50, 0, 0, 0, 0, 0.25, 0.01);
    // set position to starting position
    resetEncoderPosition();
  }

  // Below are methods & their commands for simple robot operations
  // Because we need commands for when buttons are pressed, we create methods first then use
  // run(()->{method})

private void setupMotors() {
    TalonFXConfiguration configurationLeft =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(ElevatorConstants.kStatorLimit)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(ElevatorConstants.kSupplyLimit)
                    .withSupplyCurrentLimitEnable(true))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(
                        ElevatorConstants.kLeftInverted
                            ? InvertedValue.Clockwise_Positive
                            : InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withFeedback(
                new FeedbackConfigs()
                    .withSensorToMechanismRatio(ElevatorConstants.kElevatorGearing)
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor));

    TalonFXConfiguration configurationRight =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(ElevatorConstants.kStatorLimit)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(ElevatorConstants.kStatorLimit)
                    .withSupplyCurrentLimitEnable(true))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(
                        ElevatorConstants.kRightInverted
                            ? InvertedValue.Clockwise_Positive
                            : InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withFeedback(
                new FeedbackConfigs()
                    .withSensorToMechanismRatio(ElevatorConstants.kElevatorGearing)
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(
                        convertMetersToRot(ElevatorConstants.kMaxVelocity.in(MetersPerSecond)))
                    .withMotionMagicAcceleration(
                        convertMetersToRot(
                            ElevatorConstants.kMaxAcceleration.in(MetersPerSecondPerSecond))))
            .withSlot0(
                new Slot0Configs()
                    .withGravityType(GravityTypeValue.Elevator_Static)
                    .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign));

    elevatorMotorLeft.getConfigurator().apply(configurationLeft);
    elevatorMotorRight.getConfigurator().apply(configurationRight);
  }

  //Sets voltage
public void setVoltage(Voltage Volts) {
    elevatorMotorRight.setVoltage(Volts.in(Volt));
    elevatorMotorLeft.setControl(
        new Follower(elevatorMotorRight.getDeviceID(), ElevatorConstants.kFollowerInverted));
  }


  public Command setVoltage(Supplier<Voltage> volts) {
    return run(
        () -> {
          setVoltage(volts.get());
        });
  }

public void goToPosition(Distance position) {
    lastTargetHeight = position;
    elevatorMotorRight.setControl(new MotionMagicVoltage(convertMetersToRot(position.in(Meters))));
    elevatorMotorLeft.setControl(
        new Follower(elevatorMotorRight.getDeviceID(), ElevatorConstants.kFollowerInverted));
  }

  // Goes to height
  public void goToHeight(Distance targetHeight) {
    this.targetHeight = targetHeight;
    goToPosition(targetHeight);
  }

  // returns a Command to go to height
  public Command goToHeight(Supplier<Distance> targetHeight) {
    return run(
        () -> {
          double setpoint =
              MathUtil.clamp(
                  targetHeight.get().in(Meters),
                  ElevatorConstants.kElevatorMinimumHeight.in(Meters),
                  ElevatorConstants.kElevatorMaximumHeight.in(Meters));
          goToHeight(Meters.of(setpoint));
        });
  }

  public void resetEncoderPosition() {
    elevatorMotorLeft.setPosition(
        convertMetersToRot(ElevatorConstants.kElevatorStartingHeight.in(Meters)));
    elevatorMotorRight.setPosition(
        convertMetersToRot(ElevatorConstants.kElevatorStartingHeight.in(Meters)));
  }

  // Command to "home" the encoder (go to starting position & set encoder to said position)
  // Sets voltage to a constant negative voltage
  // Once current spikes (signaling motor running into resistance) & the V ~0, encoder speed is
  // about zero
  // set to 0
  public Command homeEncoder() {
    return setVoltage(() -> ElevatorConstants.kHomingVoltage)
        .until(
            () ->
                (getCurrent().in(Amp) > ElevatorConstants.kHomingCurrentThreshold.in(Amp)
                    && Math.abs(getVelocity().in(MetersPerSecond))
                        < ElevatorConstants.kHomingVelocityThreshold.in(MetersPerSecond)))
        .andThen(
            runOnce(
                () -> {
                  resetEncoderPosition();
                  isHomed = true;
                }));
  }

  public void setOnboardPID(double kP, double kI, double kD, double kS, double kG, double kV, double kA) {
    elevatorMotorRight
        .getConfigurator()
        .apply(
            new Slot0Configs()
                .withGravityType(GravityTypeValue.Elevator_Static)
                .withKP(kP)
                .withKI(kI)
                .withKD(kD)
                .withKS(kS)
                .withKG(kG)
                .withKV(kV)
                .withKA(kA)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign));
  }


  public double convertMetersToRot(double meters) {
    return meters / ElevatorConstants.kElevatorConversion.in(Meters);
  }
  // When command is run, tunable constants are created & PID controller values & target height are
  // set to said tunable values
  public Command tune() {
    TunableConstant kP = new TunableConstant("/Elevator/kP", 0);
    TunableConstant kI = new TunableConstant("/Elevator/kI", 0);
    TunableConstant kD = new TunableConstant("/Elevator/kD", 0);
    TunableConstant kG = new TunableConstant("/Elevator/kG", 0);
    TunableConstant kS = new TunableConstant("/Elevator/kS", 0);
    TunableConstant kV = new TunableConstant("/Elevator/kV", 0);
    TunableConstant kA = new TunableConstant("/Elevator/kA", 0);
    TunableConstant targetHeight = new TunableConstant("/Elevator/targetHeight", 0);

    return runOnce(
            () -> {
              setOnboardPID(kP.get(), kI.get(), kD.get(), kG.get(), kS.get(), kV.get(), kA.get());
            })
        .andThen(
            run(
                () -> {
                  goToHeight(Meters.of(targetHeight.get()));
                }));
  }

  public Distance getHeight() {
    Distance height =
    Meters.of(
        elevatorMotorRight.getPosition().getValueAsDouble()
            * ElevatorConstants.kElevatorConversion.in(Meters));
    return height;
    
  }

  public boolean elevatorIsHomed() {
    return isHomed;
  }

  public boolean inCollisionZone() {
    if (getHeight() == null) return false;
    return getHeight().compareTo(ElevatorConstants.kElevatorDangerHeight) < 0;
  }

  public boolean atSetpoint() {
    boolean atSetpoint =
        Math.abs(getHeight().in(Meters) - lastTargetHeight.in(Meters))
            < ElevatorConstants.kHeightTolerance.in(Meters);
    return atSetpoint;
  }

  public Distance getTargetHeight() {
    return targetHeight;
  }

  public Current getCurrent(){
    Current current = Amps.of(elevatorMotorRight.getStatorCurrent().getValueAsDouble());
    return current;
  }

  public LinearVelocity getVelocity(){
    LinearVelocity velocity =
        MetersPerSecond.of(
            elevatorMotorRight.getVelocity().getValueAsDouble()
                * ElevatorConstants.kElevatorConversion.in(Meters));
    return velocity;
  }

  public boolean atHeight(Distance height) {
    return Math.abs(getHeight().in(Meters) - height.in(Meters))
        < ElevatorConstants.kHeightTolerance.in(Meters);
  }
}
