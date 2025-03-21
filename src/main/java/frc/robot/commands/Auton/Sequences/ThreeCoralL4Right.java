package frc.robot.commands.Auton.Sequences;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.Lib.AdvancedPose2D;
import frc.robot.Constants.AutoAimConstants;
import frc.robot.Constants.AutoAimConstants.Alignment;
import frc.robot.Constants.AutoAimConstants.Position;
import frc.robot.Constants.AutoAimConstants.ReefStation;
import frc.robot.Constants.AutonConstants;
import frc.robot.commands.Auton.Functions.AutonAllToPosition;
import frc.robot.commands.Auton.Functions.AutonCoralCollect;
import frc.robot.commands.Auton.Functions.AutonCoralScore;
import frc.robot.commands.Auton.Functions.AutonDriveToPosition;
import frc.robot.commands.Auton.Functions.InRangeAllToPosition;
import frc.robot.commands.Auton.Functions.Wait;
import frc.robot.subsystems.CoralAffector;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ThreeCoralL4Right extends SequentialCommandGroup {
  private HashMap<ReefStation, AdvancedPose2D> reef;
  private AdvancedPose2D coralStation, initialPose;
  private final DriveTrain m_driveTrain;

  /** Creates a new ThreeCoralL4. */
  public ThreeCoralL4Right(DriveTrain driveTrain, Elevator elevator, CoralAffector coralAffector) {
    reef = AutoAimConstants.redReef;
    coralStation = AutoAimConstants.blueRightCoralStationPos;
    initialPose = AutonConstants.initialPoseBlueRight;
    m_driveTrain = driveTrain;
    
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ParallelCommandGroup(new AutonDriveToPosition(driveTrain, reef.get(ReefStation.backRight).withReefAlignment(Alignment.right, true)),
                                         new InRangeAllToPosition(elevator, coralAffector, driveTrain, Position.L4)),
                new AutonCoralScore(coralAffector),
                new ParallelCommandGroup(new AutonDriveToPosition(driveTrain, coralStation.withRobotRelativeTransformation(
                                                                                  new Translation2d(-AutoAimConstants.coralStationSideOffsetDistance,
                                                                                                    -AutoAimConstants.coralStationToRobotDistance))),
                                         new SequentialCommandGroup(new Wait(AutonConstants.awayFromReefTime), 
                                                                    new AutonAllToPosition(elevator, coralAffector, driveTrain, Position.floor),
                                                                    new InRangeAllToPosition(elevator, coralAffector, driveTrain, Position.HP))),
                new AutonCoralCollect(coralAffector), 
                new ParallelCommandGroup(new AutonDriveToPosition(driveTrain, reef.get(ReefStation.frontRight).withReefAlignment(Alignment.right, true)),
                                         new InRangeAllToPosition(elevator, coralAffector, driveTrain, Position.L4)),
                new AutonCoralScore(coralAffector),
                new ParallelCommandGroup(new AutonDriveToPosition(driveTrain, coralStation.withRobotRelativeTransformation(
                                                                                  new Translation2d(-AutoAimConstants.coralStationSideOffsetDistance,
                                                                                                    -AutoAimConstants.coralStationToRobotDistance))),
                                         new SequentialCommandGroup(new Wait(AutonConstants.awayFromReefTime), 
                                                                    new AutonAllToPosition(elevator, coralAffector, driveTrain, Position.floor),
                                                                    new InRangeAllToPosition(elevator, coralAffector, driveTrain, Position.HP))),
                new AutonCoralCollect(coralAffector), 
                new ParallelCommandGroup(new AutonDriveToPosition(driveTrain, reef.get(ReefStation.frontRight).withReefAlignment(Alignment.left, true)),
                                         new InRangeAllToPosition(elevator, coralAffector, driveTrain, Position.L4)),
                new AutonCoralScore(coralAffector),
                new ParallelCommandGroup(new AutonDriveToPosition(driveTrain, coralStation.withRobotRelativeTransformation(
                                                                                  new Translation2d(-AutoAimConstants.coralStationSideOffsetDistance,
                                                                                                    -AutoAimConstants.coralStationToRobotDistance))),
                                         new SequentialCommandGroup(new Wait(AutonConstants.awayFromReefTime), 
                                                                    new AutonAllToPosition(elevator, coralAffector, driveTrain, Position.floor),
                                                                    new InRangeAllToPosition(elevator, coralAffector, driveTrain, Position.HP))),
                new AutonCoralCollect(coralAffector), 
                new ParallelCommandGroup(new AutonDriveToPosition(driveTrain, reef.get(ReefStation.frontRight).withReefAlignment(Alignment.left, false)),
                                         new InRangeAllToPosition(elevator, coralAffector, driveTrain, Position.L3)),
                new AutonCoralScore(coralAffector),
                new AutonDriveToPosition(driveTrain, coralStation.withRobotRelativeTransformation(
                                                         new Translation2d(-AutoAimConstants.coralStationSideOffsetDistance, 
                                                                           -AutoAimConstants.coralStationToRobotDistance))));
  }

  public void setAlliance(Alliance alliance) {
    if (alliance == Alliance.Red) {
      reef = AutoAimConstants.redReef;
      coralStation = AutoAimConstants.redRightCoralStationPos;
      initialPose = AutonConstants.initialPoseRedRight;
    }

    m_driveTrain.setInitialPose(initialPose);
  }
}
