package frc.robot.commands.Auton.Sequences;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.Lib.AdvancedPose2D;
import frc.robot.Constants.AutonConstants;
import frc.robot.commands.Auton.Functions.AutonDriveToPosition;
import frc.robot.commands.Auton.Functions.SetInitialPose;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveOffLine extends SequentialCommandGroup {
  /** Creates a new DriveOffLine. */
  public DriveOffLine(DriveTrain driveTrain, Alliance alliance) {
    AdvancedPose2D initialPose = alliance == Alliance.Red ? AutonConstants.initialPoseRedBack 
                                                          : AutonConstants.initialPoseBlueBack;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new SetInitialPose(driveTrain, initialPose),
                new AutonDriveToPosition(driveTrain,initialPose.withRobotRelativeTransformation(new Translation2d(0, 1))));
  }
}
