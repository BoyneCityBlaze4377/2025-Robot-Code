package frc.robot.commands.Auton.Functions;

import edu.wpi.first.wpilibj2.command.Command;
import frc.Lib.AdvancedPose2D;
import frc.robot.subsystems.DriveTrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AdvancedPose2dDrive extends Command {
  /** Creates a new AdvancedPose2dDrive. */
  public AdvancedPose2dDrive(DriveTrain driveTrain, AdvancedPose2D currentPose, 
                             AdvancedPose2D desiredPose) {
    // Use addRequirements() here to declare subsystem dependencies.
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
    return false;
  }
}
