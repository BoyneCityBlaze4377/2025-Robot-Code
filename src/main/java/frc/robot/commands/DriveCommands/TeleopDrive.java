package frc.robot.commands.DriveCommands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;
import frc.robot.Constants.DriveConstants;

public class TeleopDrive extends Command {
  private Joystick m_Joystick;
  private DriveTrain m_driveTrain;
  private double x, y, z;

  /** 
   * Drives the robot during Teleop.
   * 
   * @param driveTrain The DriveTrain to be used.
   * @param joystick The joystick to read values from.
   */
  public TeleopDrive(Joystick joystick, DriveTrain driveTrain) {
    m_Joystick = joystick;
    m_driveTrain = driveTrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveTrain.coastAll();
    x = 0;
    y = 0;
    z = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    x = (Math.abs(m_Joystick.getY()) < DriveConstants.xyDeadband ? 0 : -m_Joystick.getY());
    y = (Math.abs(m_Joystick.getX()) < DriveConstants.xyDeadband ? 0 : m_Joystick.getX());
    z = (Math.abs(m_Joystick.getZ()) < DriveConstants.zDeadband ? 0 : m_Joystick.getZ());    
    m_driveTrain.teleopDrive(x, -y, -z);

    SmartDashboard.putNumber("POV", m_Joystick.getPOV());
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
