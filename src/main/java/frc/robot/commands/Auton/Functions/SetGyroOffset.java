package frc.robot.commands.Auton.Functions;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetGyroOffset extends InstantCommand {
  private final DriveTrain m_driveTrain;
  private final double m_offset;
  public SetGyroOffset(DriveTrain driveTrain, double offset) {
    m_driveTrain = driveTrain;
    m_offset = offset;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveTrain.setGyroOffset(m_offset);
  }
}
