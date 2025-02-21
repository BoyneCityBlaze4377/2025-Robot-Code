package frc.robot.commands.DriveCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoAimConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.AutoAimConstants.Alignment;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.VisionSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LimeLightDrive extends Command {
  private final DriveTrain m_driveTrain;
  private double xSpeed, ySpeed, rot;
  private final double targetDistance, targetOffsetDeg, targetAngle, maxHorizOutput, maxDisOutput, maxRotOutput;
  private final PIDController angleController, horizController, distanceController;
  private final Alignment alignment;
  private final VisionSubsystem m_visionSubsystem;

  /** Creates a new LimeLightDrive. */
  public LimeLightDrive(DriveTrain driveTrain, VisionSubsystem visionSubsystem, 
                        double TargetDistance,  Alignment Alignment) {
    m_driveTrain = driveTrain;
    m_visionSubsystem = visionSubsystem;

    targetDistance = TargetDistance;
    alignment = Alignment;

    horizController = new PIDController(.01, 0, 0);
    distanceController = new PIDController(.01, 0, 0);
    angleController = new PIDController(.1, 0, 0);

    horizController.setTolerance(1);
    distanceController.setTolerance(10);
    angleController.setTolerance(2);

    addRequirements(m_driveTrain);

    targetOffsetDeg = AutoAimConstants.offsetFromAlignment.get(alignment) == 0 ? 0 : 
                      1 / Math.atan(AutoAimConstants.offsetFromAlignment.get(alignment) / targetDistance);
    targetAngle = AutoAimConstants.angleFromReefStation.get(AutoAimConstants.reefStationFromAprilTagID.get(
                                                            m_visionSubsystem.getTargetID()));

    maxHorizOutput = .5;
    maxDisOutput = .5;
    maxRotOutput = .1;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ySpeed = MathUtil.clamp(horizController.calculate(m_visionSubsystem.getTX(), targetOffsetDeg), 
                            -maxHorizOutput, maxHorizOutput);
    xSpeed = MathUtil.clamp(distanceController.calculate(m_visionSubsystem.getDistanceMeasurementmm(), targetDistance * 1000), 
                            -maxDisOutput, maxDisOutput);
    rot = MathUtil.clamp(horizController.calculate(m_driveTrain.getHeading(), targetAngle), 
                         -maxRotOutput, maxRotOutput);

    m_driveTrain.autonDrive(xSpeed, ySpeed, rot);
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
    return (horizController.atSetpoint() && distanceController.atSetpoint() && angleController.atSetpoint()) 
            || m_visionSubsystem.getTargetID() == 0;
  }
}
