/* (C) Robolancers 2025 */
package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drivetrain.Drive;

public class RobotContainer {

  Drive drive;
  CommandXboxController driverController = new CommandXboxController(0);

  public RobotContainer() {

    drive = Drive.create();

    drive.setDefaultCommand(
        drive.driveSim(
            driverController.getLeftX(),
            -driverController.getLeftY(),
            driverController.getRightX()));

    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
