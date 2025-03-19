package frc.robot.commands.Auton.Sequences;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
import frc.robot.commands.Auton.Functions.Wait;
import frc.robot.subsystems.AlgaeAffector;
import frc.robot.subsystems.CoralAffector;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoL4AndProcessor extends SequentialCommandGroup {
  /** Creates a new TwoL4AndProcessor. */
  public TwoL4AndProcessor(DriveTrain driveTrain, Elevator elevator, CoralAffector coralAffector, 
                           AlgaeAffector algaeAffector, Alliance alliance) {
    HashMap<ReefStation, AdvancedPose2D> reef = alliance == Alliance.Blue ? AutoAimConstants.blueReef : AutoAimConstants.redReef;
    AdvancedPose2D coralStation = alliance == Alliance.Blue ? AutoAimConstants.blueRightCoralStationPos : AutoAimConstants.redRightCoralStationPos;
    AdvancedPose2D processor = alliance == Alliance.Blue ? FieldConstants.blueProcessor : FieldConstants.redprocessor;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ParallelCommandGroup(new AutonDriveToPosition(driveTrain, reef.get(ReefStation.backRight).withReefAlignment(Alignment.right)),
                                         new InRangeAllToPosition(elevator, coralAffector, driveTrain, Position.L4)),
                new AutonCoralScore(coralAffector),
                new ParallelCommandGroup(new AutonDriveToPosition(driveTrain, coralStation.withRobotRelativeTransformation(
                                                                                  new Translation2d(-AutoAimConstants.coralStationSideOffsetDistance, 
                                                                                                    -AutoAimConstants.coralStationToRobotDistance))),
                                         new SequentialCommandGroup(new Wait(AutonConstants.awayFromReefTime), 
                                                                    new AutonAllToPosition(elevator, coralAffector, driveTrain, Position.floor),
                                                                    new InRangeAllToPosition(elevator, coralAffector, driveTrain, Position.HP))),
                new AutonCoralCollect(coralAffector), 
                new ParallelCommandGroup(new AutonDriveToPosition(driveTrain, reef.get(ReefStation.frontRight).withReefAlignment(Alignment.right)),
                                         new InRangeAllToPosition(elevator, coralAffector, driveTrain, Position.L4)),
                new AutonCoralScore(coralAffector),
                new ParallelCommandGroup(new AutonDriveToPosition(driveTrain, reef.get(ReefStation.frontRight).withReefAlignment(Alignment.center)),
                                         new SequentialCommandGroup(new Wait(AutonConstants.awayFromReefTime), 
                                                                    new AutonAllToPosition(elevator, coralAffector, driveTrain, Position.L2algae))),
                new ParallelDeadlineGroup(new AutonAlgaeCollect(algaeAffector),
                                          new AutonDriveToPosition(driveTrain, reef.get(ReefStation.frontRight).withReefAlignment(Alignment.center)
                                                                                                               .withRobotRelativeTransformation(new Translation2d(0, -AutoAimConstants.algaePosBackset)))),
                new ParallelCommandGroup(new AutonAllToPosition(elevator, coralAffector, driveTrain, Position.processor),
                                         new SequentialCommandGroup(new AutonDriveToPosition(driveTrain, reef.get(ReefStation.frontRight).withReefAlignment(Alignment.center)),
                                                                    new AutonDriveToPosition(driveTrain, processor.withRobotRelativeTransformation(new Translation2d(0, -.5))),
                                                                    new AutonDriveToPosition(driveTrain, processor)),
                new AutonAlgaeScore(algaeAffector),
                new ParallelCommandGroup(new AutonDriveToPosition(driveTrain, coralStation.withRobotRelativeTransformation(
                                                                                  new Translation2d(-AutoAimConstants.coralStationSideOffsetDistance, 
                                                                                                    -AutoAimConstants.coralStationToRobotDistance))),
                                         new InRangeAllToPosition(elevator, coralAffector, driveTrain, Position.HP)),
                new ParallelCommandGroup(new AutonDriveToPosition(driveTrain, reef.get(ReefStation.frontRight).withReefAlignment(Alignment.left)),
                                         new SequentialCommandGroup(new AutonAllToPosition(elevator, coralAffector, driveTrain, Position.floor),
                                                                    new InRangeAllToPosition(elevator, coralAffector, driveTrain, Position.L4))),
                new AutonCoralScore(coralAffector)));
  }
}
