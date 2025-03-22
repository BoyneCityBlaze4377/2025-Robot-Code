package frc.robot.commands.Auton.Sequences;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.Lib.AdvancedPose2D;
import frc.robot.commands.Auton.Functions.RawAutonDrive;
import frc.robot.commands.Auton.Functions.SetInitialPose;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveOffLine extends SequentialCommandGroup {
  /** Creates a new DriveOffLine. */
  public DriveOffLine(DriveTrain driveTrain, AdvancedPose2D initialPose) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new SetInitialPose(driveTrain, initialPose),
                new ParallelCommandGroup(new RawAutonDrive(driveTrain, -.5, 0, 0, 3, 0))
                                                          .withTimeout(3));
  }
}
