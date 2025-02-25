package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.Lib.DPadSelector;
import frc.robot.Constants.AutoAimConstants;
import frc.robot.Constants.AutoAimConstants.Alignment;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.VisionSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlign extends Command {
  private final DriveTrain m_driveTrain;
  private double xSpeed, ySpeed, rot, targetDistance, targetOffsetDeg, targetAngle, maxHorizOutput, 
                  maxDisOutput, maxRotOutput;
  private final PIDController angleController, horizController, distanceController;
  private final DPadSelector<Alignment> alignmentSelector;
  private final Alignment alignment;
  private final VisionSubsystem m_visionSubsystem;

  /** Creates a new LimeLightDrive. */
  public AutoAlign(DriveTrain driveTrain, VisionSubsystem visionSubsystem, int POVDegrees, double TargetDistance) {
    m_driveTrain = driveTrain;
    m_visionSubsystem = visionSubsystem;

    targetDistance = TargetDistance;
    alignmentSelector = new DPadSelector<Alignment>(270,90);
    alignment = alignmentSelector.selectOutput(POVDegrees, Alignment.center, Alignment.left, Alignment.right);

    horizController = new PIDController(AutoAimConstants.horizkP, 
                                        AutoAimConstants.horizkI, 
                                        AutoAimConstants.horizkD);
    distanceController = new PIDController(AutoAimConstants.diskP, 
                                           AutoAimConstants.diskI, 
                                           AutoAimConstants.diskP);
    angleController = new PIDController(AutoAimConstants.turnkP, 
                                        AutoAimConstants.turnkI, 
                                        AutoAimConstants.turnkD);

    horizController.setTolerance(AutoAimConstants.horizkTolerance);
    distanceController.setTolerance(AutoAimConstants.diskTolerance);
    angleController.setTolerance(AutoAimConstants.turnkTolerance);

    addRequirements(m_driveTrain);

    maxHorizOutput = 3;
    maxDisOutput = 3;
    maxRotOutput = Math.PI/2;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetDistance += AutoAimConstants.LCToBumperEdgeOffsetMeters;
    targetOffsetDeg = (AutoAimConstants.offsetFromAlignment.get(alignment) == 0 ? 0 : 
                      1 / Math.atan(AutoAimConstants.offsetFromAlignment.get(alignment) 
                      / targetDistance)) + AutoAimConstants.LLDefaultOffsetDegrees;
    targetAngle = AutoAimConstants.angleFromReefStation.get(AutoAimConstants.reefStationFromAprilTagID.get(
                                                            m_visionSubsystem.getTargetID()));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ySpeed = MathUtil.clamp(horizController.calculate(m_visionSubsystem.getTX(), targetOffsetDeg), 
                            -maxHorizOutput, maxHorizOutput);
    xSpeed = MathUtil.clamp(distanceController.calculate(m_visionSubsystem.getDistanceMeasurementmm(), targetDistance * 1000), 
                            -maxDisOutput, maxDisOutput);
    rot = MathUtil.clamp(angleController.calculate(m_driveTrain.getHeading(), targetAngle), 
                         -maxRotOutput, maxRotOutput);

    m_driveTrain.autonDrive(-xSpeed, ySpeed, -rot);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.autonDrive(0, 0, 0);
    xSpeed = 0;
    ySpeed = 0;
    rot = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (horizController.atSetpoint() && distanceController.atSetpoint() && 
            angleController.atSetpoint()) || m_visionSubsystem.getTargetID() == 0;
  }
}
