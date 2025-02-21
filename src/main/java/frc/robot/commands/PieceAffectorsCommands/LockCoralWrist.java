package frc.robot.commands.PieceAffectorsCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralAffector;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LockCoralWrist extends Command {
  private final CoralAffector m_affector;

  /** Creates a new LockCoralWrist. */
  public LockCoralWrist(CoralAffector affector) {
    m_affector = affector;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_affector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_affector.lockWrist();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_affector.stopWrist();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
