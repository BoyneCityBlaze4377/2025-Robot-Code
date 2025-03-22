package frc.robot.commands.Auton.Functions;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.Lib.AdvancedPose2D;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetInitialPose extends InstantCommand {
  private final DriveTrain m_driveTrain;
  private final AdvancedPose2D m_initialPose;
  public SetInitialPose(DriveTrain driveTrain, AdvancedPose2D initialPose) {
    m_driveTrain = driveTrain;
    m_initialPose = initialPose;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveTrain.setInitialPose(m_initialPose);
    m_driveTrain.setInitialPose(m_initialPose);
    m_driveTrain.setInitialPose(m_initialPose);
    m_driveTrain.setInitialPose(m_initialPose);
    m_driveTrain.setInitialPose(m_initialPose);
  }
}
