package frc.robot.commands.DriveCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.AutoAimConstants.Alignment;
// import frc.robot.subsystems.AutoAimSubsystem;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SelectDesiredAlignment extends InstantCommand {
  // private final AutoAimSubsystem m_autoAimSubsystem;
  private final Alignment m_alignment;
  private final DriveTrain m_driveTrain;

  public SelectDesiredAlignment(DriveTrain driveTrain, Alignment alignment) {
    // m_autoAimSubsystem = autoAimSubsystem;
    m_alignment = alignment;
    m_driveTrain = driveTrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //m_autoAimSubsystem.setDesiredAlignment(m_alignment);
    m_driveTrain.setDesiredAlignment(m_alignment);
  }
}
