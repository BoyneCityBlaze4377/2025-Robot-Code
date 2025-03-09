package frc.robot.commands.Auton.Sequences;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.Lib.AdvancedPose2D;
import frc.robot.Constants.AutoAimConstants;
import frc.robot.Constants.AutoAimConstants.Alignment;
import frc.robot.Constants.AutoAimConstants.Position;
import frc.robot.Constants.AutoAimConstants.ReefStation;
import frc.robot.commands.AllToSetPosition;
import frc.robot.commands.Auton.Functions.AutonCoralCollect;
import frc.robot.commands.Auton.Functions.AutonCoralScore;
import frc.robot.commands.Auton.Functions.InRangeAllToPosition;
import frc.robot.commands.DriveCommands.DriveToPosition;
import frc.robot.subsystems.CoralAffector;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ThreeCoralL4Right extends SequentialCommandGroup {
  /** Creates a new ThreeCoralL4. */
  public ThreeCoralL4Right(DriveTrain driveTrain, Elevator elevator, CoralAffector coralAffector, Alliance alliance) {
    HashMap<ReefStation, AdvancedPose2D> reef = alliance == Alliance.Blue ? AutoAimConstants.blueReef : AutoAimConstants.redReef;
    AdvancedPose2D coralStation = alliance == Alliance.Blue ? AutoAimConstants.blueRightCoralStationPos : AutoAimConstants.redRightCoralStationPos;
    
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ParallelCommandGroup(new DriveToPosition(driveTrain, reef.get(ReefStation.backRight).withReefAlignment(Alignment.right)),
                                         new InRangeAllToPosition(elevator, coralAffector, driveTrain, Position.L4)),
                new AutonCoralScore(coralAffector),
                new ParallelCommandGroup(new DriveToPosition(driveTrain, coralStation.withRobotRelativeTransformation(new Translation2d(-Units.inchesToMeters(24), 0))),
                                         new SequentialCommandGroup(new AllToSetPosition(elevator, coralAffector, Position.floor),
                                                                    new InRangeAllToPosition(elevator, coralAffector, driveTrain, Position.HP))),
                new AutonCoralCollect(coralAffector), 
                new ParallelCommandGroup(new DriveToPosition(driveTrain, reef.get(ReefStation.frontRight).withReefAlignment(Alignment.right)),
                                         new InRangeAllToPosition(elevator, coralAffector, driveTrain, Position.L4)),
                new AutonCoralScore(coralAffector),
                new ParallelCommandGroup(new DriveToPosition(driveTrain, coralStation.withRobotRelativeTransformation(new Translation2d(-Units.inchesToMeters(24), 0))),
                                         new SequentialCommandGroup(new AllToSetPosition(elevator, coralAffector, Position.floor),
                                                                    new InRangeAllToPosition(elevator, coralAffector, driveTrain, Position.HP))),
                new AutonCoralCollect(coralAffector), 
                new ParallelCommandGroup(new DriveToPosition(driveTrain, reef.get(ReefStation.frontRight).withReefAlignment(Alignment.left)),
                                         new InRangeAllToPosition(elevator, coralAffector, driveTrain, Position.L4)),
                new AutonCoralScore(coralAffector),
                new ParallelCommandGroup(new DriveToPosition(driveTrain, coralStation.withRobotRelativeTransformation(new Translation2d(-Units.inchesToMeters(24), 0))),
                                         new SequentialCommandGroup(new AllToSetPosition(elevator, coralAffector, Position.floor),
                                                                    new InRangeAllToPosition(elevator, coralAffector, driveTrain, Position.HP))),
                new AutonCoralCollect(coralAffector), 
                new ParallelCommandGroup(new DriveToPosition(driveTrain, reef.get(ReefStation.frontRight).withReefAlignment(Alignment.left)),
                                         new InRangeAllToPosition(elevator, coralAffector, driveTrain, Position.L3)),
                new AutonCoralScore(coralAffector),
                new DriveToPosition(driveTrain, coralStation.withRobotRelativeTransformation(new Translation2d(-Units.inchesToMeters(24), 0))));
  }
}
