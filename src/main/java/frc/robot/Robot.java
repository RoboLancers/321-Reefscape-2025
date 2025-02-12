/* (C) Robolancers 2025 */
package frc.robot;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.ReefAlign;
import frc.robot.util.VirtualSubsystem;

@Logged
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  @Logged(name = "RobotContainer")
  private final RobotContainer m_robotContainer;

  // @Logged(name = "AssetsRobotPose")
  // public Pose2d robotPose = new Pose2d();

  // @Logged(name = "AssetsZeroComponentPoses")
  // public Pose3d[] zeroComponentPoses =
  //     new Pose3d[] {new Pose3d(), new Pose3d(), new Pose3d(), new Pose3d()};

  public Robot() {
    m_robotContainer = new RobotContainer();
    Epilogue.bind(this);
  }

  @Override
  public void robotPeriodic() {
    VirtualSubsystem.periodicAll();

    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    // auto init is the first time that alliance is guaranteed to be resolved, according to memory
    ReefAlign.loadReefAlignmentPoses();

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    // alliance is also available here, for testing without needing to enable auto first
    ReefAlign.loadReefAlignmentPoses();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
