package frc.robot.commands.DriveCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.Lib.AdvancedPose2D;
import frc.robot.subsystems.AutoAimSubsystem;
import frc.robot.subsystems.DriveTrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAimDrive extends Command {
  private final DriveTrain m_driveTrain;
  private final AutoAimSubsystem m_autoAimSubsystem;
  private  AdvancedPose2D desiredPose;

  /** Creates a new AutoAimDrive. */
  public AutoAimDrive(DriveTrain driveTrain, AutoAimSubsystem autoAimSubsystem) {
    m_driveTrain = driveTrain;
    m_autoAimSubsystem = autoAimSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    desiredPose = m_autoAimSubsystem.getDesiredPose().withReefAlignment(m_autoAimSubsystem.getDesiredAlignment(), false);
    m_driveTrain.setPIDSetpoints(desiredPose.getX(), desiredPose.getY(), desiredPose.getRotation().getRadians());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    desiredPose = m_autoAimSubsystem.getDesiredPose().withReefAlignment(m_autoAimSubsystem.getDesiredAlignment(), false);
    m_driveTrain.setPIDSetpoints(desiredPose.getX(), desiredPose.getY(), desiredPose.getRotation().getRadians());
    
    m_driveTrain.PIDDrive();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!interrupted) m_driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_driveTrain.atSetpoints();
  }
}
