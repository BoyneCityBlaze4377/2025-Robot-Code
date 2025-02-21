package frc.robot.commands.ClimberCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeAffector;
import frc.robot.subsystems.Climber;

public class Climb extends Command {
  private final Climber m_climber;
  private final AlgaeAffector m_algaeWrist;

  /** Creates a new Climb. */
  public Climb(Climber climber, AlgaeAffector algaeWrist) {
    m_climber = climber;
    m_algaeWrist = algaeWrist;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_climber, m_algaeWrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_climber.climb();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
