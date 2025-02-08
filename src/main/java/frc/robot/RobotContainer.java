/* (C) Robolancers 2025 */
package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.drivetrain.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
@Logged
public class RobotContainer {
  // The robot's subsystems
  // private final CANRollerSubsystem rollerSubsystem = new CANRollerSubsystem();

  private Drivetrain drivetrain = new Drivetrain();

  // The driver's controller
  private final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.kDriverController);

  // The operator's controller
  private final CommandXboxController manipulatorController =
      new CommandXboxController(OperatorConstants.kManipulatorController);

  // The autonomous chooser
  private SendableChooser<Command> autoChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Set up command bindings
    configureBindings();

    SmartDashboard.putNumber("TranslationSpeed", 3);
    SmartDashboard.putNumber("RotationSpeed", 4);
  }

  // Set the options to show up in the Dashboard for selecting auto modes. If you
  // add additional auto modes you can add additional lines here with
  // autoChooser.addOption;

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Set the A button to run the "RollerCommand" command with a fixed
    // value ejecting the gamepiece while the button is held

    // before
    // manipulatorController
    //     .a()
    //     .whileTrue(
    //         new RollerCommand(() -> RollerConstants.kRollerEject, () -> 0,
    // rollerSubsystem));

    // Set the default command for the drive subsystem to an instance of the
    // DriveCommand with the values provided by the joystick axes on the driver
    // controller. The Y axis of the controller is inverted so that pushing the
    // stick away from you (a negative value) drives the robot forwards (a positive
    // value). Similarly for the X axis where we need to flip the value so the
    // joystick matches the WPILib convention of counter-clockwise positive

    drivetrain.setDefaultCommand(
        drivetrain.driveFieldRelativeCommand(
            () -> {
              double a = SmartDashboard.getNumber("TranslationSpeed", 3);
              return -withDeadband(driverController.getLeftY(), 0.05) * a;
            },
            () ->
                -withDeadband(driverController.getLeftX(), 0.05)
                    * SmartDashboard.getNumber("TranslationSpeed", 3),
            () -> {
              double b = SmartDashboard.getNumber("RotationSpeed", 4);
              return -withDeadband(driverController.getRightX(), 0.05) * b;
            }));

    driverController.a().onTrue(Commands.runOnce(() -> drivetrain.resetPose(new Pose2d())));

    // Set the default command for the roller subsystem to an instance of
    // RollerCommand with the values provided by the triggers on the operator
    // controller
    // rollerSubsystem.setDefaultCommand(
    //     new RollerCommand(
    //         () -> manipulatorController.getRightTriggerAxis(),
    //         () -> manipulatorController.getLeftTriggerAxis(),
    //         rollerSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }

  public double withDeadband(double val, double deadband) {
    return Math.abs(val) <= deadband ? 0 : val;
  }
}
