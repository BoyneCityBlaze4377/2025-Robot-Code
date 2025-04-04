// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auton.Sequences;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoAimConstants;
import frc.robot.Constants.AutoAimConstants.Alignment;
import frc.robot.Constants.AutoAimConstants.Position;
import frc.robot.Constants.AutoAimConstants.ReefStation;
import frc.robot.Constants.AutonConstants;
import frc.robot.commands.Auton.Functions.AutonCoralScore;
import frc.robot.commands.Auton.Functions.AutonDriveToPosition;
import frc.robot.commands.Auton.Functions.InRangeAllToPosition;
import frc.robot.commands.Auton.Functions.SetInitialPose;
import frc.robot.commands.Auton.Functions.Wait;
import frc.robot.subsystems.CoralAffector;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BackAndScoreRed extends SequentialCommandGroup {
  /** Creates a new BackAndScoreRed. */
  public BackAndScoreRed(DriveTrain driveTrain, Elevator elevator, CoralAffector coralAffector, Alignment coralAlignment) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new SetInitialPose(driveTrain, AutonConstants.initialPoseRedBack),
                new Wait(2),
                new ParallelCommandGroup(new AutonDriveToPosition(driveTrain, AutoAimConstants.redReef.get(ReefStation.back).withReefAlignment(coralAlignment, true)),
                                         new InRangeAllToPosition(elevator, coralAffector, driveTrain, Position.L4)).withTimeout(10),
                new AutonCoralScore(coralAffector));
  }
}
