package frc.robot.commands.PieceAffectorsCommands;

import edu.wpi.first.math.MathUtil;
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

    addRequirements();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    input = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    input = Math.abs(m_stick.getRawAxis(1)) < .1 ? 0 : m_stick.getRawAxis(1);
    input = MathUtil.clamp(input, AffectorConstants.maxCoralWristDownSpeed, AffectorConstants.maxCoralWristUpSpeed);

    if (input == 0) {
      m_coralAffector.lockWrist();
    } else {
      m_coralAffector.moveWrist(input);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_coralAffector.stopWrist();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
