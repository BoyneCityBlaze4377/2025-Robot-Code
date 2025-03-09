package frc.robot.commands.Auton.Functions;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RawAutonDrive extends Command {
  private final DriveTrain m_driveTrain;
  private double xSpeed, ySpeed, rot;
  private final double m_driveTime, m_rotTime;
  private final Timer timer;

  /** Creates a new RawAutonDrive. */
  public RawAutonDrive(DriveTrain driveTrain, double xSpeedMPS, double ySpeedMPS, 
                       double rotRadPerSec, double driveTime, double turnTime) {
    m_driveTrain = driveTrain;
    timer = new Timer();

    xSpeed = xSpeedMPS;
    ySpeed = ySpeedMPS;
    rot = rotRadPerSec;
    m_driveTime = driveTime;
    m_rotTime = turnTime;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.get() >= m_driveTime) {
      xSpeed = 0;
      ySpeed = 0;
    }

    if (timer.get() >= m_rotTime) {
      rot = 0;
    }

    m_driveTrain.autonDrive(xSpeed, ySpeed, rot);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    m_driveTrain.stop();

    xSpeed = 0;
    ySpeed = 0;
    rot = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() >= m_driveTime && timer.get() >= m_rotTime;
  }
}
