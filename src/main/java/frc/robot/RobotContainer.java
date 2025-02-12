/* (C) Robolancers 2025 */
package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.HomingCommands;
import frc.robot.commands.ReefAlign;
import frc.robot.commands.StationAlign;
import frc.robot.subsystems.AlgaeSuperstructure;
import frc.robot.subsystems.AlgaeSuperstructure.AlgaeSetpoint;
import frc.robot.subsystems.CoralSuperstructure;
import frc.robot.subsystems.CoralSuperstructure.CoralScorerSetpoint;
import frc.robot.subsystems.SuperstructureVisualizer;
import frc.robot.subsystems.algaeIntakePivot.AlgaeIntakePivot;
import frc.robot.subsystems.algaeIntakeRollers.AlgaeIntakeRollers;
import frc.robot.subsystems.coralendeffector.CoralEndEffector;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevatorarm.ElevatorArm;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.MathUtils;
import frc.robot.util.ReefPosition;
import java.util.function.DoubleSupplier;

@Logged
public class RobotContainer {
  private SwerveDrive drivetrain = SwerveDrive.create();
  private AlgaeIntakePivot algaePivot = AlgaeIntakePivot.create();
  private AlgaeIntakeRollers algaeRollers = AlgaeIntakeRollers.create();
  private CoralEndEffector coralEndEffector = CoralEndEffector.create();
  private ElevatorArm elevatorArm = ElevatorArm.create();
  private Elevator elevator = Elevator.create();

  private CoralSuperstructure coralSuperstructure =
      new CoralSuperstructure(elevator, elevatorArm, coralEndEffector);
  private AlgaeSuperstructure algaeSuperstructure =
      new AlgaeSuperstructure(algaePivot, algaeRollers);

  private Vision vision =
      Vision.create(
          drivetrain::getPose,
          (estimate, stdDevs) ->
              drivetrain.addVisionMeasurement(
                  estimate.estimatedPose.toPose2d(), estimate.timestampSeconds, stdDevs));

  private CommandXboxController driver = new CommandXboxController(0);
  private XboxController manipulator = new XboxController(1);

  private Trigger isSlowMode = driver.leftBumper();

  private DoubleSupplier driverForward =
      () ->
          -MathUtils.deadband(driver.getLeftY(), 0.05)
              * (isSlowMode.getAsBoolean()
                  ? 2
                  : DrivetrainConstants.kMaxLinearVelocity.in(MetersPerSecond));
  private DoubleSupplier driverStrafe =
      () ->
          -MathUtils.deadband(driver.getLeftX(), 0.05)
              * (isSlowMode.getAsBoolean()
                  ? 2
                  : DrivetrainConstants.kMaxLinearVelocity.in(MetersPerSecond));
  private DoubleSupplier driverTurn = () -> -MathUtils.deadband(driver.getRightX(), 0.05) * 5;

  // robot queued states
  private ReefPosition queuedReefPosition = ReefPosition.NONE;
  private CoralScorerSetpoint queuedSetpoint = CoralScorerSetpoint.NEUTRAL;

  private SuperstructureVisualizer stateVisualizer =
      new SuperstructureVisualizer(
          () -> elevator.getHeight(), () -> elevatorArm.getAngle(), () -> algaePivot.getAngle());

  public RobotContainer() {

    // home everything on robot start
    RobotModeTriggers.disabled()
        .negate()
        .onTrue(HomingCommands.homeEverything(elevator, algaePivot));

    // drive
    drivetrain.setDefaultCommand(drivetrain.teleopDrive(driverForward, driverStrafe, driverTurn));

    // algae default commands (stalling rollers, default algae pivot setpoint)
    algaeRollers.setDefaultCommand(algaeRollers.stallIfHasAlgae());
    algaePivot.setDefaultCommand(algaePivot.goToAngle(() -> AlgaeSetpoint.NEUTRAL.getAlgaeAngle()));

    elevator.setDefaultCommand(
        elevator.goToHeight(() -> CoralScorerSetpoint.NEUTRAL.getElevatorHeight()));
    elevatorArm.setDefaultCommand(
        elevatorArm.goToAngle(() -> CoralScorerSetpoint.NEUTRAL.getArmAngle()));
    coralEndEffector.setDefaultCommand(coralEndEffector.stallCoralIfDetected());

    // when both are about to collide, move elevator out of the way until the algae pivot is out of
    // the collision zone
    new Trigger(algaePivot::inCollisionZone)
        .and(new Trigger(elevator::inCollisionZone))
        .onTrue(
            elevator
                .goToHeight(() -> ElevatorConstants.kElevatorDangerHeight.plus(Meters.of(0.1)))
                // .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                .until(new Trigger(algaePivot::inCollisionZone).negate()));

    configureBindings();
  }

  private void configureBindings() {
    // driver controls
    // score coral / flip off algae
    driver.y().toggleOnTrue(algaeSuperstructure.prepareClimb());
    driver.a().onTrue(algaeSuperstructure.climb());

    // algae intake/outtake
    driver.b().whileTrue(algaeSuperstructure.intakeAlgae());
    driver.x().whileTrue(algaeSuperstructure.outtakeAlgae());

    /**
     * Pressing right trigger down all the way performs translation-align/to-setpoint, while
     * pressing it slightly performs the rotation align
     *
     * <p>Driver has override over translation-align/to-setpoint
     */
    new Trigger(() -> driver.getRightTriggerAxis() >= 0.8)
        .whileTrue(
            ReefAlign.alignToReef(drivetrain, () -> queuedReefPosition)
                .onlyWhile(
                    () ->
                        Math.hypot(driverForward.getAsDouble(), driverStrafe.getAsDouble()) <= 0.05)
                .asProxy()
                .repeatedly()
                .alongWith(coralSuperstructure.goToSetpoint(() -> queuedSetpoint)));

    new Trigger(() -> driver.getRightTriggerAxis() > 0.05 && driver.getRightTriggerAxis() < 0.8)
        .whileTrue(ReefAlign.rotateToNearestReefTag(drivetrain, driverForward, driverStrafe));

    driver
        .rightBumper()
        .whileTrue(
            StationAlign.rotateToNearestStationTag(drivetrain, driverForward, driverStrafe)
                .alongWith(coralSuperstructure.feedCoral()));

    // manip controls
    // 1 to 4 - right side L1-L4
    // 5 to 8 - left side L1-L4
    // 9 to 10 - algae low / high

    manipTrigger(1)
        .onTrue(
            Commands.runOnce(
                () -> {
                  queuedReefPosition = ReefPosition.RIGHT;
                  queuedSetpoint = CoralScorerSetpoint.L1;
                }));

    manipTrigger(2)
        .onTrue(
            Commands.runOnce(
                () -> {
                  queuedReefPosition = ReefPosition.RIGHT;
                  queuedSetpoint = CoralScorerSetpoint.L2;
                }));

    manipTrigger(3)
        .onTrue(
            Commands.runOnce(
                () -> {
                  queuedReefPosition = ReefPosition.RIGHT;
                  queuedSetpoint = CoralScorerSetpoint.L3;
                }));

    manipTrigger(4)
        .onTrue(
            Commands.runOnce(
                () -> {
                  queuedReefPosition = ReefPosition.RIGHT;
                  queuedSetpoint = CoralScorerSetpoint.L4;
                }));

    manipTrigger(5)
        .onTrue(
            Commands.runOnce(
                () -> {
                  queuedReefPosition = ReefPosition.LEFT;
                  queuedSetpoint = CoralScorerSetpoint.L1;
                }));

    manipTrigger(6)
        .onTrue(
            Commands.runOnce(
                () -> {
                  queuedReefPosition = ReefPosition.LEFT;
                  queuedSetpoint = CoralScorerSetpoint.L2;
                }));

    manipTrigger(7)
        .onTrue(
            Commands.runOnce(
                () -> {
                  queuedReefPosition = ReefPosition.LEFT;
                  queuedSetpoint = CoralScorerSetpoint.L3;
                }));

    manipTrigger(8)
        .onTrue(
            Commands.runOnce(
                () -> {
                  queuedReefPosition = ReefPosition.LEFT;
                  queuedSetpoint = CoralScorerSetpoint.L4;
                }));

    manipTrigger(9)
        .onTrue(
            Commands.runOnce(
                () -> {
                  queuedReefPosition = ReefPosition.ALGAE;
                  queuedSetpoint = CoralScorerSetpoint.ALGAE_LOW;
                }));

    manipTrigger(10)
        .onTrue(
            Commands.runOnce(
                () -> {
                  queuedReefPosition = ReefPosition.ALGAE;
                  queuedSetpoint = CoralScorerSetpoint.ALGAE_HIGH;
                }));
  }

  private Trigger manipTrigger(int button) {
    return new Trigger(() -> manipulator.getRawButton(button));
  }

  public Command getAutonomousCommand() {
    return Commands.none();
  }
}
