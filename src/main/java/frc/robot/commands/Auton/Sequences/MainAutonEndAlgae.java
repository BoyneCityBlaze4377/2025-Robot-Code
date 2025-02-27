package frc.robot.commands.Auton.Sequences;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.Lib.AdvancedPose2D;
import frc.robot.Constants.AutoAimConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.AutoAimConstants.Alignment;
import frc.robot.Constants.AutoAimConstants.Position;
import frc.robot.Constants.AutoAimConstants.ReefStation;
import frc.robot.commands.AllToSetPosition;
import frc.robot.commands.Auton.Functions.AutonAlgaeCollect;
import frc.robot.commands.Auton.Functions.AutonAlgaeScore;
import frc.robot.commands.Auton.Functions.AutonAllToPosition;
import frc.robot.commands.Auton.Functions.AutonAutoAlign;
import frc.robot.commands.Auton.Functions.AutonCoralCollect;
import frc.robot.commands.Auton.Functions.AutonCoralScore;
import frc.robot.commands.Auton.Functions.AutonDriveToPosition;
import frc.robot.commands.Auton.Functions.FirstRobotRelativeAutonDrive;
import frc.robot.commands.Auton.Functions.InRangeAllToPosition;
import frc.robot.commands.Auton.Functions.RobotRelativeAutonDrive;
import frc.robot.commands.Auton.Functions.SetDriveTrainPose;
import frc.robot.commands.Auton.Functions.TimedAlgaeCollect;
import frc.robot.commands.Auton.Functions.VisionTargetDetected;
import frc.robot.commands.ElevatorCommands.ElevatorToPosition;
import frc.robot.subsystems.AlgaeAffector;
import frc.robot.subsystems.CoralAffector;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.VisionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MainAutonEndAlgae extends SequentialCommandGroup {
  private HashMap<ReefStation, AdvancedPose2D> reef;
  /** Creates a new MainAuton. 
   * 
   * @param endAlgaeOrCoral true for algae, false for coral
  */
  public MainAutonEndAlgae(DriveTrain driveTrain, Elevator elevator, 
                   CoralAffector coralAffector, AlgaeAffector algaeAffector, 
                   VisionSubsystem visionSubsystem, Alliance alliance) {
    reef = alliance == Alliance.Blue ? AutoAimConstants.blueReef : AutoAimConstants.redReef;
    
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new FirstRobotRelativeAutonDrive(driveTrain, 0, 4, 0, 
                                                 Units.inchesToMeters(110), driveTrain.getHeading(), visionSubsystem,
                                                 ReefStation.backRight),
                new ParallelCommandGroup(new AutonAutoAlign(driveTrain, visionSubsystem, .003, Alignment.left)),
                                         new InRangeAllToPosition(elevator, coralAffector, driveTrain, Position.L4),
                new SequentialCommandGroup(new AutonCoralScore(coralAffector), new SetDriveTrainPose(driveTrain, reef.get(ReefStation.backRight)
                                                                                                                 .withRobotRelativeTransformation(new Translation2d(0, 
                                                                                                                 -AutoAimConstants.leftCoralReefOffset)))),
                new ParallelCommandGroup(new SequentialCommandGroup(new AllToSetPosition(elevator, coralAffector, Position.floor), 
                                                                    new InRangeAllToPosition(elevator, coralAffector, driveTrain, Position.HP)),
                                         new AutonDriveToPosition(driveTrain, reef.get(ReefStation.backRight)
                                                                              .withRobotRelativeTransformation(new Translation2d(0, 
                                                                              -AutoAimConstants.leftCoralReefOffset)), 
                                                                  AutoAimConstants.blueRightCoralStationPos.withRobotRelativeTransformation(
                                                                      new Translation2d(0, -AutoAimConstants.coralStationSideOffsetDistance)),
                                                                  4, Math.PI/2)),
                new AutonCoralCollect(coralAffector),
                new ParallelRaceGroup(new AutonDriveToPosition(driveTrain, AutoAimConstants.blueRightCoralStationPos.withRobotRelativeTransformation(
                                              new Translation2d(0, -AutoAimConstants.coralStationSideOffsetDistance)), 
                                              reef.get(ReefStation.frontRight), 0, 0),
                                      new VisionTargetDetected(visionSubsystem, ReefStation.frontRight)),
                new ParallelCommandGroup(new AutonAutoAlign(driveTrain, visionSubsystem, .003, Alignment.right),
                        new InRangeAllToPosition(elevator, coralAffector, driveTrain, Position.L4)),
                new AutonCoralScore(coralAffector),
                new ParallelCommandGroup(new AutonAllToPosition(elevator, coralAffector, Position.floor),
                                         new AutonDriveToPosition(driveTrain, reef.get(ReefStation.frontRight), new AdvancedPose2D((alliance == Alliance.Blue ? 
                                                                                                                                    FieldConstants.blueRightStartAlgae 
                                                                                                                                    : FieldConstants.blueRightStartAlgae)
                                                                                                                    .withRobotRelativeTransformation(new Translation2d(-.5, 0)).rotateBy(
                                                                                                                                                     new Rotation2d(Math.atan(reef.get(ReefStation.frontRight).minus(
                                                                                                                                                     (alliance == Alliance.Blue ? FieldConstants.blueRightStartAlgae : 
                                                                                                                                                     FieldConstants.blueRightStartAlgae)).getY() / 
                                                                                                                                                     reef.get(ReefStation.frontRight).minus((alliance == Alliance.Blue ? 
                                                                                                                                                     FieldConstants.blueRightStartAlgae : FieldConstants.blueRightStartAlgae)).getX())))),
                                         3, Math.PI/3)), 
                new ParallelDeadlineGroup(new SequentialCommandGroup(new TimedAlgaeCollect(algaeAffector, .3), 
                                                                     new ParallelCommandGroup(new ElevatorToPosition(elevator, Position.L1), 
                                                                     new AutonAlgaeCollect(algaeAffector)))), 
                new RobotRelativeAutonDrive(driveTrain, 0, 3, 
                                            0, .6, 360),
                new AutonDriveToPosition(driveTrain, (alliance == Alliance.Blue 
                                                      ? FieldConstants.blueRightStartAlgae 
                                                      : FieldConstants.blueRightStartAlgae), 
                                         FieldConstants.blueProcessor.withVector(new Rotation2d().fromDegrees(90),
                                                                                 new Translation2d(0, .5), new Rotation2d().fromDegrees(-90)),
                                         4, Math.PI/2),
                new ParallelCommandGroup(new RobotRelativeAutonDrive(driveTrain, 0, 1, 0, .5, 360),
                                         new AutonAlgaeScore(algaeAffector)));
  }
}
