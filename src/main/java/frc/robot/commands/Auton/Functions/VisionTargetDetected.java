package frc.robot.commands.Auton.Functions;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoAimConstants;
import frc.robot.Constants.AutoAimConstants.ReefStation;
import frc.robot.subsystems.AutoAimSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class VisionTargetDetected extends Command {
  private final AutoAimSubsystem m_subsystem;
  private final double targetID;
  /** Creates a new VisionTargetDetected. */
  public VisionTargetDetected(AutoAimSubsystem subsystem, ReefStation targetStation) {
    m_subsystem = subsystem;
    targetID = DriverStation.getAlliance().get() == Alliance.Blue ? AutoAimConstants.blueReefIDsFromStation.get(targetStation)
                                                                  : AutoAimConstants.redReefIDsFromStation.get(targetStation);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_subsystem.getTargetID() == targetID;
  }
}
