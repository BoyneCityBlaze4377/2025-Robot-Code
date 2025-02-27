package frc.robot.commands.Auton.Functions;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.Lib.AdvancedPose2D;
import frc.robot.Constants.AutoAimConstants;
import frc.robot.subsystems.DriveTrain;

public class AutonDriveToPosition extends Command {
  private double xSpeed, ySpeed, rot, targetHeading, turnError, relativeAngle, 
                 initialTurnValue, time, desiredDriveAngle, desiredDistance, v;
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
    rot = omegaRadiansPerSecond;
    desiredDistance = currentPosition.getTranslation().getDistance(desiredPose.getTranslation());
    targetHeading = desiredPose.getRotation().getDegrees();
    m_driveTrain = driveTrain;
    m_timer = new Timer();
    time = desiredDistance / vMetersPerSecond;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialTurnValue = m_driveTrain.getHeading();
    m_driveTrain.brakeAll();
    m_driveTrain.setOrientation(true);

    Math.copySign(targetHeading, rot);

    m_timer.reset();
    m_timer.start();
    m_driveTrain.setInRange(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    relativeAngle = MathUtil.inputModulus(m_driveTrain.getHeading() - initialTurnValue, 
                                          -360, 360);
    turnError = MathUtil.inputModulus(targetHeading + Math.copySign(relativeAngle, rot), 
                                      -360, 360);

    if (m_timer.get() >= time) {
      xSpeed = 0;
      ySpeed = 0;
    }

    if (Math.abs(turnError) <= 5) {
      targetHeading = m_driveTrain.getHeading();
      rot = 0;
      turnError = 0;
    }

    m_driveTrain.autonDrive(-xSpeed, ySpeed, rot);
    m_driveTrain.setInRange(m_timer.get() <= (desiredDistance - AutoAimConstants.inRangeThreshold) 
                                              / v);
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
    return m_timer.get() >= time && Math.abs(turnError) <= 2;
  }
}
