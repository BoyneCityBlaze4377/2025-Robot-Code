package frc.robot.commands.Auton.Functions;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeAffector;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TimedAlgaeCollect extends Command {
  private final AlgaeAffector m_affector;
  private final Timer timer;
  private final double m_time;

  /** Creates a new AutonAlgaeCollect. */
  public TimedAlgaeCollect(AlgaeAffector affector, double time) {
    m_affector = affector;
    timer = new Timer();
    m_time = time;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_affector);
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
    m_affector.collect();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    m_affector.stopAffector();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() >= m_time;
  }
}
