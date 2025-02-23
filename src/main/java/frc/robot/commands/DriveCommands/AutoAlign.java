package frc.robot.commands.DriveCommands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.Lib.DPadSelector;
import frc.robot.Constants.AutoAimConstants.Alignment;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.VisionSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlign extends Command {
  private final DPadSelector<Command> alignmentSelector = new DPadSelector<Command> (270, 90);
  private final DriveTrain m_driveTrain;
  private final VisionSubsystem m_visionSubsystem;
  private final Joystick m_stick;

  /** Creates a new AutoAlign. */
  public AutoAlign(DriveTrain driveTrain, VisionSubsystem visionSubsystem, Joystick stick) {
    m_driveTrain = driveTrain;
    m_visionSubsystem = visionSubsystem;
    m_stick = stick;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrain);
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
