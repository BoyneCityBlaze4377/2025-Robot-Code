package frc.robot.commands.DriveCommands;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.Lib.AdvancedPose2D;
import frc.Lib.AutoAimHelpers;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoAimDrive extends SequentialCommandGroup {
  /** Creates a new AutoAimDrive. */
  public AutoAimDrive(DriveTrain driveTrain, AdvancedPose2D desiredPose, Alliance alliance) {
    ArrayList<AdvancedPose2D> optimalPath = AutoAimHelpers.getOptimalPath(new AdvancedPose2D(driveTrain.getPose()), desiredPose, alliance);
    Command[] pathCommands = new Command[optimalPath.toArray().length];
    for (int i = 0; i < pathCommands.length; i++) {
      pathCommands[i] = new DriveToPosition(driveTrain, optimalPath.get(i));
    }
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(pathCommands);
  }
}
