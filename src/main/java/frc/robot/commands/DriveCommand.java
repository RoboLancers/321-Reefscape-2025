/* (C) Robolancers 2025 */
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.*;
import java.util.function.DoubleSupplier;

// Command to drive the robot with joystick inputs
public class DriveCommand extends Command {
  private final DoubleSupplier xSpeed;
  private final DoubleSupplier zRotation;
  private final Drivetrain drivetrain;

  //   private final CANDriveSubsystem driveSubsystem;

  // Constructor. Runs only once when the command is first created.
  public DriveCommand(DoubleSupplier xSpeed, DoubleSupplier zRotation, Drivetrain drivetrain) {
    // Save parameters to local variables for use later
    this.xSpeed = xSpeed;
    this.zRotation = zRotation;
    this.drivetrain = drivetrain;

    // Declare subsystems required by this command. This should include any
    // subsystem this command sets and output of
    addRequirements(this.drivetrain);
  }

  // Runs each time the command is scheduled.
  @Override
  public void initialize() {}

  // Runs every cycle while the command is scheduled (~50 times per second)
  @Override
  public void execute() {
    drivetrain.driveFieldRelativeCommand(xSpeed, null, null);
  }

  // Runs each time the command ends via isFinished or being interrupted.
  @Override
  public void end(boolean isInterrupted) {}

  // Runs every cycle while the command is scheduled to check if the command is
  // finished
  @Override
  public boolean isFinished() {
    // Return false to indicate that this command never ends. It can be interrupted
    // by another command needing the same subsystem.
    return false;
  }
}
