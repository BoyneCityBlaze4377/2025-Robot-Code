package frc.robot;

import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.IOConstants;
import au.grapplerobotics.CanBridge;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  // private double periodicCounter = 0;

  private RobotContainer m_robotContainer;

  public Robot() {
    CanBridge.runTCP();
  }
  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
    Shuffleboard.selectTab(IOConstants.ConfigTab.getTitle());
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    //m_robotContainer.robotRelative();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    // m_robotContainer.setDriveOrientation(true);
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    Shuffleboard.selectTab(IOConstants.AutonTab.getTitle());
    // m_robotContainer.setDriveTrainPoseEstimate();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    Shuffleboard.selectTab(IOConstants.TeleopTab.getTitle());
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    // m_robotContainer.setDriveTrainPoseEstimate();
    m_robotContainer.setDriveOrientation(true);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // m_robotContainer.setDriveOrientation(true);
    // if (periodicCounter > 20) m_robotContainer.setDriveTrainPoseEstimate(); periodicCounter = 0;
    // periodicCounter++;
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
