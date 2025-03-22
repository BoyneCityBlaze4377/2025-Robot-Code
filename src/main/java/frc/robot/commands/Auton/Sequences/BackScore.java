package frc.robot.commands.Auton.Sequences;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.Lib.AdvancedPose2D;
import frc.robot.Constants.AutoAimConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.AutoAimConstants.Position;
import frc.robot.Constants.AutoAimConstants.ReefStation;
import frc.robot.commands.AllToSetPosition;
import frc.robot.commands.Auton.Functions.AutonCoralScore;
import frc.robot.commands.Auton.Functions.AutonStopDriveTrain;
import frc.robot.commands.Auton.Functions.RawAutonDrive;
import frc.robot.commands.Auton.Functions.SetInitialPose;
import frc.robot.commands.DriveCommands.StopDriveTrain;
import frc.robot.subsystems.CoralAffector;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BackScore extends SequentialCommandGroup {
  /** Creates a new DriveOffLine. */
  public BackScore(DriveTrain driveTrain, AdvancedPose2D initialPose, Elevator elevator, CoralAffector coralAffector) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new SetInitialPose(driveTrain, initialPose),
                new ParallelCommandGroup(new RawAutonDrive(driveTrain, -.5, 0, 0, 
                                         (FieldConstants.autonLineDistance - FieldConstants.blueReefCenterPos.getX() 
                                          + AutoAimConstants.centerOfReefToRobotDistance) * 2,
                                         0),
                                         new AllToSetPosition(elevator, coralAffector, Position.L4)).withTimeout(6.5),
                new AutonCoralScore(coralAffector),
                new RawAutonDrive(driveTrain, .5, 0, 0, 2, 0).withTimeout(1.5),
                new AutonStopDriveTrain(driveTrain),
                new AllToSetPosition(elevator, coralAffector, Position.floor));
  }
}
