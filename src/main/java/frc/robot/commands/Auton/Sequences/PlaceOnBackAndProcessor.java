package frc.robot.commands.Auton.Sequences;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.Lib.AdvancedPose2D;
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
import frc.robot.commands.Auton.Functions.Wait;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.AlgaeAffector;
import frc.robot.subsystems.CoralAffector;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceOnBackAndProcessor extends SequentialCommandGroup {
  private HashMap<ReefStation, AdvancedPose2D> reef;
  private AdvancedPose2D processor, initialPose;
  private final DriveTrain m_driveTrain;

  /** Creates a new PlaceOnBackAndProcessor. */
  public PlaceOnBackAndProcessor(DriveTrain driveTrain, CoralAffector coralAffector, Elevator elevator,
                                 AlgaeAffector algaeAffector, Alignment coralAlignment) {
    reef = AutoAimConstants.blueReef;
    processor = FieldConstants.blueProcessor;
    initialPose = AutonConstants.initialPoseBlueBack;
    m_driveTrain = driveTrain;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ParallelCommandGroup(new AutonDriveToPosition(driveTrain, reef.get(ReefStation.back).withReefAlignment(coralAlignment, true))),
                                         new InRangeAllToPosition(elevator, coralAffector, driveTrain, Position.L4),
                new AutonCoralScore(coralAffector),
                new ParallelCommandGroup(new AutonDriveToPosition(driveTrain, reef.get(ReefStation.back).withReefAlignment(Alignment.center, false)),
                                         new SequentialCommandGroup(new Wait(AutonConstants.awayFromReefTime), 
                                                                    new AutonAllToPosition(elevator, coralAffector, driveTrain, Position.L2algae))),
                new ParallelDeadlineGroup(new AutonAlgaeCollect(algaeAffector),
                                          new AutonDriveToPosition(driveTrain, reef.get(ReefStation.back).withReefAlignment(Alignment.center, false)
                                                                                                         .withRobotRelativeTransformation(new Translation2d(0, -AutoAimConstants.algaePosBackset)))),
                new ParallelCommandGroup(new AutonAllToPosition(elevator, coralAffector, driveTrain, Position.processor),
                                         new SequentialCommandGroup(new AutonDriveToPosition(driveTrain, reef.get(ReefStation.back).withReefAlignment(Alignment.center, false)),
                                                                    new AutonDriveToPosition(driveTrain, processor.withRobotRelativeTransformation(new Translation2d(0, -.5))),
                                                                    new AutonDriveToPosition(driveTrain, processor))),
                new AutonAlgaeScore(algaeAffector));
  }

  public void setAlliance(Alliance alliance) {
    if (alliance == Alliance.Red) {
      reef = AutoAimConstants.redReef;
      processor = FieldConstants.redprocessor;
      initialPose = AutonConstants.initialPoseRedBack;
    }

    m_driveTrain.setInitialPose(initialPose);
  }
}
