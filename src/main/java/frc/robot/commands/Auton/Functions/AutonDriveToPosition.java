package frc.robot.commands.Auton.Functions;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.Lib.AdvancedPose2D;
import frc.robot.Constants.AutoAimConstants;
import frc.robot.subsystems.DriveTrain;

public class AutonDriveToPosition extends Command {
  private double xSpeed, ySpeed, rot, targetHeading, time, desiredDriveAngle, desiredDistance, v;
  private final DriveTrain m_driveTrain;
  private final Timer m_timer;
  /** Creates a new AutonDrive. */
  public AutonDriveToPosition(DriveTrain driveTrain, AdvancedPose2D currentPosition, 
                              AdvancedPose2D desiredPose, double vMetersPerSecond, 
                              double omegaRadiansPerSecond) {
    desiredDriveAngle = Math.atan(currentPosition.minus(desiredPose).getY() /
                                  currentPosition.minus(desiredPose).getX());
    ySpeed = vMetersPerSecond * Math.cos(desiredDriveAngle);
    xSpeed = vMetersPerSecond * Math.sin(desiredDriveAngle);
    v = vMetersPerSecond;
    rot = Math.abs(omegaRadiansPerSecond);
    desiredDistance = currentPosition.getTranslation().getDistance(desiredPose.getTranslation());
    targetHeading = desiredPose.getRotation().getDegrees();
    m_driveTrain = driveTrain;
    m_timer = new Timer();
    time = Math.abs(desiredDistance / vMetersPerSecond);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveTrain.brakeAll();
    m_driveTrain.setOrientation(true);

    rot *= targetHeading < m_driveTrain.getHeading() ? -1 : 1;

    m_timer.reset();
    m_timer.start();
    m_driveTrain.setInRange(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_timer.get() >= time) {
      xSpeed = 0;
      ySpeed = 0;
    }

    if (Math.abs(targetHeading - m_driveTrain.getHeading()) <= 2) {
      targetHeading = m_driveTrain.getHeading();
      rot = 0;
    }

    m_driveTrain.autonDrive(-xSpeed, ySpeed, rot);
    m_driveTrain.setInRange(m_timer.get() <= (desiredDistance - AutoAimConstants.inRangeThreshold) / v);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    targetHeading = m_driveTrain.getHeading();
    rot = 0;
    xSpeed = 0;
    ySpeed = 0;

    m_timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.get() >= time && Math.abs(targetHeading - m_driveTrain.getHeading()) <= 3;
  }
}
