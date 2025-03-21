package frc.robot.commands.Auton.Sequences;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.Lib.AdvancedPose2D;
import frc.robot.Constants.AutoAimConstants;
import frc.robot.Constants.AutonConstants;
import frc.robot.Constants.AutoAimConstants.Alignment;
import frc.robot.Constants.AutoAimConstants.Position;
import frc.robot.Constants.AutoAimConstants.ReefStation;
import frc.robot.commands.Auton.Functions.AutonAllToPosition;
import frc.robot.commands.Auton.Functions.AutonCoralCollect;
import frc.robot.commands.Auton.Functions.AutonCoralScore;
import frc.robot.commands.Auton.Functions.AutonDriveToPosition;
import frc.robot.commands.Auton.Functions.InRangeAllToPosition;
import frc.robot.subsystems.CoralAffector;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FourCoralL3Left extends SequentialCommandGroup {
  private HashMap<ReefStation, AdvancedPose2D> reef;
  private AdvancedPose2D coralStation, initialPose;
  private final DriveTrain m_driveTrain;

  /** Creates a new FourCoralL3. */
  public FourCoralL3Left(DriveTrain driveTrain, CoralAffector coralAffector, Elevator elevator) {
    reef = AutoAimConstants.blueReef;
    coralStation = AutoAimConstants.blueRightCoralStationPos;
    initialPose = AutonConstants.initialPoseBlueLeft;
    m_driveTrain = driveTrain;
    
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ParallelCommandGroup(new AutonDriveToPosition(driveTrain, reef.get(ReefStation.backLeft).withReefAlignment(Alignment.left, false)),
                                         new InRangeAllToPosition(elevator, coralAffector, driveTrain, Position.L3)),
                new AutonCoralScore(coralAffector),
                new ParallelCommandGroup(new AutonDriveToPosition(driveTrain, coralStation.withRobotRelativeTransformation(
                                                                                  new Translation2d(AutoAimConstants.coralStationSideOffsetDistance, 
                                                                                                   -AutoAimConstants.coralStationToRobotDistance))),
                                         new SequentialCommandGroup(new AutonAllToPosition(elevator, coralAffector, driveTrain, Position.floor), 
                                                                    new InRangeAllToPosition(elevator, coralAffector, driveTrain, Position.HP))),
                new AutonCoralCollect(coralAffector),
                new ParallelCommandGroup(new AutonDriveToPosition(driveTrain, reef.get(ReefStation.backLeft).withReefAlignment(Alignment.right, false)),
                                         new InRangeAllToPosition(elevator, coralAffector, driveTrain, Position.L3)), 
                new AutonCoralScore(coralAffector),
                new ParallelCommandGroup(new AutonDriveToPosition(driveTrain, coralStation.withRobotRelativeTransformation(
                                                                                  new Translation2d(AutoAimConstants.coralStationSideOffsetDistance, 
                                                                                                   -AutoAimConstants.coralStationToRobotDistance))),
                                         new SequentialCommandGroup(new AutonAllToPosition(elevator, coralAffector, driveTrain, Position.floor), 
                                                                    new InRangeAllToPosition(elevator, coralAffector, driveTrain, Position.HP))),
                new AutonCoralCollect(coralAffector),
                new ParallelCommandGroup(new AutonDriveToPosition(driveTrain, reef.get(ReefStation.frontLeft).withReefAlignment(Alignment.left, false)),
                                         new InRangeAllToPosition(elevator, coralAffector, driveTrain, Position.L3)), 
                new AutonCoralScore(coralAffector),
                new ParallelCommandGroup(new AutonDriveToPosition(driveTrain, coralStation.withRobotRelativeTransformation(
                                                                                  new Translation2d(AutoAimConstants.coralStationSideOffsetDistance, 
                                                                                                   -AutoAimConstants.coralStationToRobotDistance))),
                                         new SequentialCommandGroup(new AutonAllToPosition(elevator, coralAffector, driveTrain, Position.floor), 
                                                                    new InRangeAllToPosition(elevator, coralAffector, driveTrain, Position.HP))),
                new AutonCoralCollect(coralAffector),
                new ParallelCommandGroup(new AutonDriveToPosition(driveTrain, reef.get(ReefStation.frontLeft).withReefAlignment(Alignment.right, false)),
                                         new InRangeAllToPosition(elevator, coralAffector, driveTrain, Position.L3)),
                new AutonCoralScore(coralAffector),
                new ParallelCommandGroup(new AutonDriveToPosition(driveTrain, coralStation.withRobotRelativeTransformation(
                                                                                  new Translation2d(AutoAimConstants.coralStationSideOffsetDistance, 
                                                                                                   -AutoAimConstants.coralStationToRobotDistance))),
                                         new SequentialCommandGroup(new AutonAllToPosition(elevator, coralAffector, driveTrain, Position.floor), 
                                                                    new InRangeAllToPosition(elevator, coralAffector, driveTrain, Position.HP))),
                new AutonCoralCollect(coralAffector));
  }

  public void setAlliance(Alliance alliance) {
    if (alliance == Alliance.Red) {
      reef = AutoAimConstants.redReef;
      coralStation = AutoAimConstants.redRightCoralStationPos;
      initialPose = AutonConstants.initialPoseRedLeft;
    }

    m_driveTrain.setInitialPose(initialPose);
  }
}
