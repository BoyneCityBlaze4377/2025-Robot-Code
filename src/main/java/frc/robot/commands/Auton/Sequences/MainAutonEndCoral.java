package frc.robot.commands.Auton.Sequences;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.Lib.AdvancedPose2D;
import frc.robot.Constants.AutoAimConstants;
import frc.robot.Constants.AutoAimConstants.Alignment;
import frc.robot.Constants.AutoAimConstants.Position;
import frc.robot.Constants.AutoAimConstants.ReefStation;
import frc.robot.commands.AllToSetPosition;
import frc.robot.commands.Auton.Functions.AutonAutoAlign;
import frc.robot.commands.Auton.Functions.AutonCoralCollect;
import frc.robot.commands.Auton.Functions.AutonCoralScore;
import frc.robot.commands.Auton.Functions.AutonDrive;
import frc.robot.commands.Auton.Functions.AutonDriveToPosition;
import frc.robot.commands.Auton.Functions.FirstAutonDrive;
import frc.robot.commands.Auton.Functions.InRangeAllToPosition;
import frc.robot.commands.Auton.Functions.RobotRelativeAutonDrive;
import frc.robot.commands.Auton.Functions.SetDriveTrainPose;
import frc.robot.commands.Auton.Functions.VisionTargetDetected;
import frc.robot.subsystems.AlgaeAffector;
import frc.robot.subsystems.CoralAffector;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.VisionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MainAutonEndCoral extends SequentialCommandGroup {
  private HashMap<ReefStation, AdvancedPose2D> reef;
  /** Creates a new MainAuton. 
   * 
   * @param endAlgaeOrCoral true for algae, false for coral
  */
  public MainAutonEndCoral(DriveTrain driveTrain, Elevator elevator, CoralAffector coralAffector, 
                           VisionSubsystem visionSubsystem, Alliance alliance) {
    reef = alliance == Alliance.Blue ? AutoAimConstants.blueReef : AutoAimConstants.redReef;
    
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new FirstAutonDrive(driveTrain, 180, 4, Math.PI * 2, 
                               Units.inchesToMeters(104),120),
                new ParallelCommandGroup(new AutonAutoAlign(driveTrain, visionSubsystem, .01, Alignment.left)),
                                         new InRangeAllToPosition(elevator, coralAffector, driveTrain, Position.L4),
                new SequentialCommandGroup(new AutonCoralScore(coralAffector), new SetDriveTrainPose(driveTrain, reef.get(ReefStation.backRight)
                                                                                                     .withRobotRelativeTransformation(new Translation2d(0, 
                                                                                                     -AutoAimConstants.leftCoralReefOffset)))),
                new RobotRelativeAutonDrive(driveTrain, 0, 4, 0, .3, 0),
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
                new ParallelCommandGroup(new AutonAutoAlign(driveTrain, visionSubsystem, .01, Alignment.right),
                                         new InRangeAllToPosition(elevator, coralAffector, driveTrain, Position.L4)),
                new AutonCoralScore(coralAffector),
                new ParallelCommandGroup(new SequentialCommandGroup(new AllToSetPosition(elevator, coralAffector, Position.floor), 
                                                                    new InRangeAllToPosition(elevator, coralAffector, driveTrain, Position.HP)),
                                         new AutonDriveToPosition(driveTrain, reef.get(ReefStation.frontRight)
                                                                              .withRobotRelativeTransformation(new Translation2d(0, 
                                                                              -AutoAimConstants.rightCoralReefOffset)), 
                                                                  AutoAimConstants.blueRightCoralStationPos.withRobotRelativeTransformation(
                                                                      new Translation2d(0, -AutoAimConstants.coralStationSideOffsetDistance)),
                                                                  4, Math.PI/2)),
                new AutonCoralCollect(coralAffector),
                new ParallelRaceGroup(new AutonDriveToPosition(driveTrain, AutoAimConstants.blueRightCoralStationPos.withRobotRelativeTransformation(
                                                               new Translation2d(0, -AutoAimConstants.coralStationSideOffsetDistance)), 
                                                               reef.get(ReefStation.frontRight), 0, 0),
                                      new VisionTargetDetected(visionSubsystem, ReefStation.frontRight)),
                new ParallelCommandGroup(new AutonAutoAlign(driveTrain, visionSubsystem, .01, Alignment.left),
                                         new InRangeAllToPosition(elevator, coralAffector, driveTrain, Position.L4)),
                new AutonCoralScore(coralAffector));
  }
}
