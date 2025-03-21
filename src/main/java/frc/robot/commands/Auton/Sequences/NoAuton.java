package frc.robot.commands.Auton.Sequences;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.Lib.AdvancedPose2D;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class NoAuton extends SequentialCommandGroup {
  /** Creates a new NoAuton. */
  public NoAuton(DriveTrain driveTrain, AdvancedPose2D initialPose) {
    driveTrain.setInitialPose(initialPose);

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands();
  }
}
