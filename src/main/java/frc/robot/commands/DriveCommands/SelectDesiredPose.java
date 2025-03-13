package frc.robot.commands.DriveCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.Lib.AdvancedPose2D;
import frc.robot.subsystems.AutoAimSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SelectDesiredPose extends InstantCommand { 
  private final AdvancedPose2D m_desiredPose;
  private final AutoAimSubsystem m_autoAimSubsystem;

  public SelectDesiredPose(AutoAimSubsystem autoAimSubsystem, AdvancedPose2D desiredPose) {
    m_desiredPose = desiredPose;
    m_autoAimSubsystem = autoAimSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_autoAimSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_autoAimSubsystem.setDesiredPose(m_desiredPose);
  }
}
