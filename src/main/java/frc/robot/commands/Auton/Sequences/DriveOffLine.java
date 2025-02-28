package frc.robot.commands.Auton.Sequences;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Auton.Functions.RobotRelativeAutonDrive;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveOffLine extends SequentialCommandGroup {
  /** Creates a new DriveOffLine. */
  public DriveOffLine(DriveTrain driveTrain) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new RobotRelativeAutonDrive(driveTrain, 0, 1, 
                                            0, 1, 0));
  }
}
