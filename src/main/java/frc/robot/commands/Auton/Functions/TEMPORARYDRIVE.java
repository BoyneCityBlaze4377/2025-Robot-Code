package frc.robot.commands.Auton.Functions;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TEMPORARYDRIVE extends Command {
  private final DriveTrain m_driveTrain;
  private final double SPEED;
  /** Creates a new TEMPORARYDRIVE. */
  public TEMPORARYDRIVE(DriveTrain driveTrain, double speed) {
    m_driveTrain = driveTrain;
    SPEED = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveTrain.setOrientation(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveTrain.teleopDrive(SPEED, 0, 0);
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
