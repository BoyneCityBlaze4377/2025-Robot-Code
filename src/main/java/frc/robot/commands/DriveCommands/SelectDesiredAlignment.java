package frc.robot.commands.DriveCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.AutoAimConstants.Alignment;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SelectDesiredAlignment extends InstantCommand {
  private final DriveTrain m_driveTrain;
  private final Alignment desired;
  public SelectDesiredAlignment(DriveTrain driveTrain, Alignment desiredAlignment) {
    m_driveTrain = driveTrain;
    desired = desiredAlignment;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveTrain.setDesiredAlignment(desired);
  }
}
