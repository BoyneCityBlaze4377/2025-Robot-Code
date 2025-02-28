package frc.robot.commands.Auton.Sequences;

import java.lang.reflect.Field;
import java.util.HashMap;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.Lib.AdvancedPose2D;
import frc.robot.Constants.AutoAimConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.AutoAimConstants.Alignment;
import frc.robot.Constants.AutoAimConstants.Position;
import frc.robot.Constants.AutoAimConstants.ReefStation;
import frc.robot.commands.Auton.Functions.AutonAlgaeCollect;
import frc.robot.commands.Auton.Functions.AutonAlgaeScore;
import frc.robot.commands.Auton.Functions.AutonAllToPosition;
import frc.robot.commands.Auton.Functions.AutonAutoAlign;
import frc.robot.commands.Auton.Functions.AutonCoralScore;
import frc.robot.commands.Auton.Functions.AutonDriveToPosition;
import frc.robot.commands.Auton.Functions.FirstRobotRelativeAutonDrive;
import frc.robot.commands.Auton.Functions.InRangeAllToPosition;
import frc.robot.commands.Auton.Functions.RawAutonDrive;
import frc.robot.commands.Auton.Functions.RobotRelativeAutonDrive;
import frc.robot.commands.Auton.Functions.SetDriveTrainPose;
import frc.robot.commands.Auton.Functions.StopDriveTrain;
import frc.robot.commands.Auton.Functions.Wait;
import frc.robot.subsystems.AlgaeAffector;
import frc.robot.subsystems.CoralAffector;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.VisionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Score1L4AndProcessor extends SequentialCommandGroup {
    private final HashMap<ReefStation, AdvancedPose2D> reef;
    private final AdvancedPose2D processor, station;

  /** Creates a new Score1L4AndProcessor. */
  public Score1L4AndProcessor(DriveTrain driveTrain, Elevator elevator, CoralAffector coralAffector, 
                              AlgaeAffector algaeAffector, VisionSubsystem visionSubsystem, Alliance alliance) {
    reef = alliance == Alliance.Blue ? AutoAimConstants.blueReef : AutoAimConstants.redReef;
    processor = alliance == Alliance.Blue ? FieldConstants.blueProcessor : FieldConstants.redprocessor;
    station = alliance == Alliance.Blue ? AutoAimConstants.blueRightCoralStationPos : AutoAimConstants.redRightCoralStationPos;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new FirstRobotRelativeAutonDrive(driveTrain, 0, 4, 0, 
                                                 Units.inchesToMeters(110), driveTrain.getHeading(), visionSubsystem,
                                                 ReefStation.backRight),
                new ParallelCommandGroup(new AutonAutoAlign(driveTrain, visionSubsystem, .01, Alignment.left)),
                                         new InRangeAllToPosition(elevator, coralAffector, driveTrain, Position.L4),
                new SequentialCommandGroup(new AutonCoralScore(coralAffector), new SetDriveTrainPose(driveTrain, reef.get(ReefStation.backRight)
                                                                                                                 .withRobotRelativeTransformation(new Translation2d(0, 
                                                                                                                 -AutoAimConstants.leftCoralReefOffset)))),
                new ParallelCommandGroup(new RawAutonDrive(driveTrain, 0, .3, Math.PI, .3, 180, false),
                                         new SequentialCommandGroup(new Wait(.1), new AutonAllToPosition(elevator, coralAffector, Position.L3algae))),
                new ParallelDeadlineGroup(new AutonAlgaeCollect(algaeAffector), 
                                          new SequentialCommandGroup(new AutonAutoAlign(driveTrain, visionSubsystem, .02, Alignment.center),
                                                                     new RobotRelativeAutonDrive(driveTrain, 0, -2, 0, 1, 0))),
                new ParallelCommandGroup(new RobotRelativeAutonDrive(driveTrain, 0, -2, 0, 1, 0),
                                         new AutonAllToPosition(elevator, coralAffector, Position.floor)),
                new ParallelCommandGroup(new AutonDriveToPosition(driveTrain, reef.get(ReefStation.backRight).withRobotRelativeTransformation(new Translation2d(-.6, 0)),
                                                                  processor.withRobotRelativeTransformation(new Translation2d(-1, 0)), 
                                                                  4, Math.PI/2)), 
                                         new AutonAllToPosition(elevator, coralAffector, Position.L1),
                new ParallelCommandGroup(new RobotRelativeAutonDrive(driveTrain, 0, 2, 0, 1, 0),
                                         new SequentialCommandGroup(new Wait(.25), new AutonAlgaeScore(algaeAffector))),
                new StopDriveTrain(driveTrain));
  }
}
