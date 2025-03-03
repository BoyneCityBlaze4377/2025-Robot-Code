package frc.robot.commands.Auton.Sequences;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoAimConstants.Position;
import frc.robot.commands.Auton.Functions.AutonAllToPosition;
import frc.robot.commands.Auton.Functions.AutonCoralScore;
import frc.robot.commands.Auton.Functions.AutonDrive;
import frc.robot.commands.Auton.Functions.STOP;
import frc.robot.commands.Auton.Functions.StopDriveTrain;
import frc.robot.commands.Auton.Functions.TEMPORARYDRIVE;
import frc.robot.commands.Auton.Functions.Wait;
import frc.robot.subsystems.CoralAffector;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TemporaryAuton extends SequentialCommandGroup {

  /** Creates a new TemporaryAuton. */
  public TemporaryAuton(DriveTrain driveTrain, Elevator elevator, CoralAffector coralAffector) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new TEMPORARYDRIVE(driveTrain, -.5).withTimeout(1),
                new STOP(driveTrain),
                new AutonAllToPosition(elevator, coralAffector, Position.L4),
                new TEMPORARYDRIVE(driveTrain, -2).withTimeout(2),
                new STOP(driveTrain),
                new AutonCoralScore(coralAffector),
                new TEMPORARYDRIVE(driveTrain, .5).withTimeout(1),
                new STOP(driveTrain),
                new AutonAllToPosition(elevator, coralAffector, Position.floor));
  }
}
