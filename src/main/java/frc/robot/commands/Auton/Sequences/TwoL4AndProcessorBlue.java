package frc.robot.commands.Auton.Sequences;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.Lib.AdvancedPose2D;
import frc.robot.Constants.AutoAimConstants;
import frc.robot.Constants.AutonConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.AutoAimConstants.Alignment;
import frc.robot.Constants.AutoAimConstants.Position;
import frc.robot.Constants.AutoAimConstants.ReefStation;
import frc.robot.commands.Auton.Functions.AutonAlgaeCollect;
import frc.robot.commands.Auton.Functions.AutonAlgaeScore;
import frc.robot.commands.Auton.Functions.AutonAllToPosition;
import frc.robot.commands.Auton.Functions.AutonCoralCollect;
import frc.robot.commands.Auton.Functions.AutonCoralScore;
import frc.robot.commands.Auton.Functions.AutonDriveToPosition;
import frc.robot.commands.Auton.Functions.InRangeAllToPosition;
import frc.robot.commands.Auton.Functions.SetInitialPose;
import frc.robot.commands.Auton.Functions.Wait;
import frc.robot.subsystems.AlgaeAffector;
import frc.robot.subsystems.CoralAffector;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoL4AndProcessorBlue extends SequentialCommandGroup {
  /** Creates a new TwoL4AndProcessor. */
  public TwoL4AndProcessorBlue(DriveTrain driveTrain, Elevator elevator, CoralAffector coralAffector, AlgaeAffector algaeAffector) {
    //HashMap<ReefStation, AdvancedPose2D> reef = AutoAimConstants.blueReef;
    AdvancedPose2D coralStation = AutoAimConstants.blueRightCoralStationPos;
    AdvancedPose2D processor = FieldConstants.blueProcessor;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new SetInitialPose(driveTrain, AutonConstants.initialPoseBlueRight),
                new ParallelCommandGroup(new AutonDriveToPosition(driveTrain, AutoAimConstants.blueReef.get(ReefStation.backRight).withReefAlignment(Alignment.right, true)),
                                         new InRangeAllToPosition(elevator, coralAffector, driveTrain, Position.L4)),
                new AutonCoralScore(coralAffector),
                new ParallelCommandGroup(new AutonDriveToPosition(driveTrain, coralStation.withRobotRelativeTransformation(
                                                                                  new Translation2d(-AutoAimConstants.coralStationSideOffsetDistance, 
                                                                                                    -AutoAimConstants.coralStationToRobotDistance))),
                                         new SequentialCommandGroup(new Wait(AutonConstants.awayFromReefTime), 
                                                                    new AutonAllToPosition(elevator, coralAffector, driveTrain, Position.floor),
                                                                    new InRangeAllToPosition(elevator, coralAffector, driveTrain, Position.HP))),
                new AutonCoralCollect(coralAffector), 
                new ParallelCommandGroup(new AutonDriveToPosition(driveTrain, AutoAimConstants.blueReef.get(ReefStation.frontRight).withReefAlignment(Alignment.right, true)),
                                         new InRangeAllToPosition(elevator, coralAffector, driveTrain, Position.L4)),
                new AutonCoralScore(coralAffector),
                new ParallelCommandGroup(new AutonDriveToPosition(driveTrain, AutoAimConstants.blueReef.get(ReefStation.frontRight).withReefAlignment(Alignment.center, false)),
                                         new SequentialCommandGroup(new Wait(AutonConstants.awayFromReefTime), 
                                                                    new AutonAllToPosition(elevator, coralAffector, driveTrain, Position.L2algae))),
                new ParallelDeadlineGroup(new AutonAlgaeCollect(algaeAffector),
                                          new AutonDriveToPosition(driveTrain, AutoAimConstants.blueReef.get(ReefStation.frontRight).withReefAlignment(Alignment.center, false)
                                                                                                               .withRobotRelativeTransformation(new Translation2d(0, -AutoAimConstants.algaePosBackset)))),
                new ParallelCommandGroup(new AutonAllToPosition(elevator, coralAffector, driveTrain, Position.processor),
                                         new SequentialCommandGroup(new AutonDriveToPosition(driveTrain, AutoAimConstants.blueReef.get(ReefStation.frontRight).withReefAlignment(Alignment.center, false)),
                                                                    new AutonDriveToPosition(driveTrain, processor.withRobotRelativeTransformation(new Translation2d(0, -.5))),
                                                                    new AutonDriveToPosition(driveTrain, processor)),
                new AutonAlgaeScore(algaeAffector),
                new ParallelCommandGroup(new AutonDriveToPosition(driveTrain, coralStation.withRobotRelativeTransformation(
                                                                                  new Translation2d(-AutoAimConstants.coralStationSideOffsetDistance, 
                                                                                                    -AutoAimConstants.coralStationToRobotDistance))),
                                         new InRangeAllToPosition(elevator, coralAffector, driveTrain, Position.HP)),
                new ParallelCommandGroup(new AutonDriveToPosition(driveTrain, AutoAimConstants.blueReef.get(ReefStation.frontRight).withReefAlignment(Alignment.left, true)),
                                         new SequentialCommandGroup(new AutonAllToPosition(elevator, coralAffector, driveTrain, Position.floor),
                                                                    new InRangeAllToPosition(elevator, coralAffector, driveTrain, Position.L4))),
                new AutonCoralScore(coralAffector)));
  }
}
