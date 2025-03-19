package frc.robot.commands.PieceAffectorsCommands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AffectorConstants;
import frc.robot.subsystems.CoralAffector;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralWristOverride extends Command {
  private final CoralAffector m_coralAffector;
  private final Joystick m_stick;
  private double input;

  /** Creates a new CoralWrist. */
  public CoralWristOverride(CoralAffector coralAffector, Joystick stick) {
    m_coralAffector = coralAffector;
    m_stick = stick;
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(m_coralAffector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    input = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    input = m_stick.getRawAxis(0) * AffectorConstants.wristOverrideSpeed;
    if (Math.abs(input) < .2) {
      m_coralAffector.overrideLockWrist();
    } else {
      m_coralAffector.overrideWrist(-input);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_coralAffector.overrideLockWrist();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
