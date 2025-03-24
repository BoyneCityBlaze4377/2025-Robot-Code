package frc.robot.commands.DriveCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAimDrive extends Command {
  private final DriveTrain m_driveTrain;
  /** Creates a new AutoAimDrive. */
  public AutoAimDrive(DriveTrain driveTrain) {
    m_driveTrain = driveTrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveTrain.setAutoAimDriveSetpoints();
    m_driveTrain.displayTrajectory();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveTrain.setAutoAimDriveSetpoints();
    m_driveTrain.PIDDrive();
    m_driveTrain.displayTrajectory();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!interrupted) m_driveTrain.stop();
    m_driveTrain.unDisplayTrajectory();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_driveTrain.atSetpoints();
  }
}
