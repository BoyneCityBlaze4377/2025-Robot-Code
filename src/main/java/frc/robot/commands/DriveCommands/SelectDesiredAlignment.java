package frc.robot.commands.DriveCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.AutoAimConstants.Alignment;
import frc.robot.subsystems.AutoAimSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SelectDesiredAlignment extends InstantCommand {
  private final AutoAimSubsystem m_autoAimSubsystem;
  private final Alignment m_alignment;

  public SelectDesiredAlignment(AutoAimSubsystem autoAimSubsystem, Alignment alignment) {
    m_autoAimSubsystem = autoAimSubsystem;
    m_alignment = alignment;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_autoAimSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_autoAimSubsystem.setDesiredAlignment(m_alignment);
  }
}
