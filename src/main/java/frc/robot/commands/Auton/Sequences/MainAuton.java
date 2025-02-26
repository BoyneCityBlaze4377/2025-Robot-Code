package frc.robot.commands.Auton.Sequences;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
import frc.robot.commands.Auton.Functions.FirstRobotRelativeAutonDrive;
import frc.robot.commands.Auton.Functions.InRangeAllToPosition;
import frc.robot.commands.Auton.Functions.RobotRelativeAutonDrive;
import frc.robot.subsystems.AlgaeAffector;
import frc.robot.subsystems.CoralAffector;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.VisionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MainAuton extends SequentialCommandGroup {
  private HashMap<ReefStation, AdvancedPose2D> reef = DriverStation.getAlliance().get() == Alliance.Blue ?
                                                      AutoAimConstants.blueReef : AutoAimConstants.redReef;
  /** Creates a new MainAuton. */
  public MainAuton(DriveTrain driveTrain, Elevator elevator, 
                   CoralAffector coralAffector, AlgaeAffector algaeAffector, 
                   VisionSubsystem visionSubsystem) {
    
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new FirstRobotRelativeAutonDrive(driveTrain, 0, 4, 0, 
                                    Units.inchesToMeters(100), driveTrain.getHeading(), visionSubsystem,
                                    ReefStation.backRight),
                new ParallelCommandGroup(new AutonAutoAlign(driveTrain, visionSubsystem, AutoAimConstants.centerOfReefToRobotDistance, Alignment.left)),
                                         new InRangeAllToPosition(elevator, coralAffector, driveTrain, Position.L4),
                new AutonCoralScore(coralAffector),
                new ParallelCommandGroup(new AllToSetPosition(elevator, coralAffector, Position.floor),
                                         //To coral station
                                         new AutonDriveToPosition(driveTrain, reef.get(ReefStation.backRight), 
                                                                  AutoAimConstants.blueRightCoralStationPos.withRobotRelativeTransformation(
                                                                      new Translation2d(0, -AutoAimConstants.coralStationSideOffsetDistance)),
                                                                  4, Math.PI/2)),
                                         new AutonCoralCollect(coralAffector));
  }
}
