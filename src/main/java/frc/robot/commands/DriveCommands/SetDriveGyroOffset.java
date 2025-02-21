package frc.robot.commands.DriveCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveTrain;

public class SetDriveGyroOffset extends InstantCommand {
  private final DriveTrain m_driveTrain;
  private double offset;

  /**
   * Sets the offset of the DriveTrain's Gryo.
   * 
   * @param driveTrain The DriveTrain to have its offset set.
   * @param offsetDegrees The number of degrees to offset by.
   */
  public SetDriveGyroOffset(DriveTrain driveTrain, double offsetDegrees) {
    m_driveTrain = driveTrain;
    offset = offsetDegrees;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveTrain.setGyroOffset(offset);
  }
}
