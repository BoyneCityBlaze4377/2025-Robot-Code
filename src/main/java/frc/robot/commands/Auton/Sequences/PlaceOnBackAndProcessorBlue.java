package frc.robot.commands.Auton.Sequences;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoAimConstants;
import frc.robot.Constants.AutoAimConstants.Alignment;
import frc.robot.Constants.AutoAimConstants.Position;
import frc.robot.Constants.AutoAimConstants.ReefStation;
import frc.robot.Constants.AutonConstants;
import frc.robot.commands.Auton.Functions.AutonAlgaeCollect;
import frc.robot.commands.Auton.Functions.AutonAlgaeScore;
import frc.robot.commands.Auton.Functions.AutonAllToPosition;
import frc.robot.commands.Auton.Functions.AutonCoralScore;
import frc.robot.commands.Auton.Functions.AutonDriveToPosition;
import frc.robot.commands.Auton.Functions.InRangeAllToPosition;
import frc.robot.commands.Auton.Functions.SetInitialPose;
import frc.robot.commands.Auton.Functions.Wait;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.AlgaeAffector;
import frc.robot.subsystems.CoralAffector;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceOnBackAndProcessorBlue extends SequentialCommandGroup {
  /** Creates a new PlaceOnBackAndProcessor. */
  public PlaceOnBackAndProcessorBlue(DriveTrain driveTrain, CoralAffector coralAffector, Elevator elevator,
                                 AlgaeAffector algaeAffector, Alignment coralAlignment) {
    //HashMap<ReefStation, AdvancedPose2D> reef = AutoAimConstants.blueReef;
    // AdvancedPose2D processor = FieldConstants.blueProcessor;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new SetInitialPose(driveTrain, AutonConstants.initialPoseBlueBack),
                new ParallelCommandGroup(new AutonDriveToPosition(driveTrain, AutoAimConstants.blueReef.get(ReefStation.back).withReefAlignment(coralAlignment, true))),
                                         new InRangeAllToPosition(elevator, coralAffector, driveTrain, Position.L4),
                new AutonCoralScore(coralAffector),
                new ParallelCommandGroup(new AutonDriveToPosition(driveTrain, AutoAimConstants.blueReef.get(ReefStation.back).withReefAlignment(Alignment.center, false)),
                                         new SequentialCommandGroup(new Wait(AutonConstants.awayFromReefTime), 
                                                                    new AutonAllToPosition(elevator, coralAffector, driveTrain, Position.L2algae))),
                new ParallelDeadlineGroup(new AutonAlgaeCollect(algaeAffector),
                                          new AutonDriveToPosition(driveTrain, AutoAimConstants.blueReef.get(ReefStation.back).withReefAlignment(Alignment.center, false))),
                new ParallelCommandGroup(new AutonAllToPosition(elevator, coralAffector, driveTrain, Position.processor),
                                         new SequentialCommandGroup(new AutonDriveToPosition(driveTrain, AutoAimConstants.blueReef.get(ReefStation.back).withReefAlignment(Alignment.center, false)),
                                                                    new AutonDriveToPosition(driveTrain, FieldConstants.blueProcessor.withRobotRelativeTransformation(new Translation2d(0, -AutoAimConstants.algaePosBackset))),
                                                                    new AutonDriveToPosition(driveTrain, FieldConstants.blueProcessor))),
                new AutonAlgaeScore(algaeAffector));
  }
}
