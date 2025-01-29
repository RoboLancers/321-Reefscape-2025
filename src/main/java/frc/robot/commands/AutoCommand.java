/* (C) Robolancers 2025 */
package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.*;

// Command to run the robot at 1/2 power for 1 second in autonomous
public class AutoCommand extends Command {
  Drivetrain drivetrain;

  private final Command autoDriveCommand;

  private Timer timer;
  private double seconds = 1.0;

  // Constructor. Runs only once when the command is first created.
  public AutoCommand(Drivetrain drivetrain) {
    // Save parameter for use later and initialize timer object.
    this.drivetrain = drivetrain;
    timer = new Timer();

    this.autoDriveCommand = drivetrain.driveFieldRelativeCommand(() -> 0.5, () -> 0.1, () -> 0.1);

    // Declare subsystems required by this command. This should include any
    // subsystem this command sets and output of
    addRequirements(drivetrain);
  }

  // Runs each time the command is scheduled. For this command, we handle starting
  // the timer.
  @Override
  public void initialize() {
    // start timer, uses restart to clear the timer as well in case this command has
    // already been run before
    timer.restart();
    autoDriveCommand.initialize();
  }

  // Runs every cycle while the command is scheduled (~50 times per second)
  @Override
  public void execute() {
    // drive at 1/2 speed
    autoDriveCommand.execute();
  }

  // Runs each time the command ends via isFinished or being interrupted.
  @Override
  public void end(boolean isInterrupted) {
    // stop drive motors
    autoDriveCommand.end(isInterrupted);
  }

  // Runs every cycle while the command is scheduled to check if the command is
  // finished
  @Override
  public boolean isFinished() {
    // check if timer exceeds seconds, when it has this will return true indicating
    // this command is finished
    return timer.get() >= seconds;
  }
}
