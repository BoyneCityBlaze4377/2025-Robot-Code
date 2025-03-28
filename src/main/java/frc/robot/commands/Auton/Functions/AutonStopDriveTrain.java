package frc.robot.commands.Auton.Functions;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveCommands.StopDriveTrain;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonStopDriveTrain extends SequentialCommandGroup {
  /** Creates a new AutonStopDriveTrain. */
  public AutonStopDriveTrain(DriveTrain driveTrain) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new StopDriveTrain(driveTrain).withTimeout(.1));
  }
}
