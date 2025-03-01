package frc.robot.commands.Auton.Functions;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoAimConstants;
import frc.robot.subsystems.DriveTrain;

public class FirstAutonDrive extends Command {
  private double xSpeed, ySpeed, rot, targetHeading, time, desiredDistance, v, tardriangle;
  private final DriveTrain m_driveTrain;
  private final Timer m_timer;
  /** Creates a new AutonDrive. */
  public FirstAutonDrive(DriveTrain driveTrain, double desiredDriveAngle, double vMetersPerSecond, 
                    double omegaRadiansPerSecond, double targetDistance, double TargetHeading) {
    xSpeed = vMetersPerSecond * Math.cos(desiredDriveAngle) / 2;
    ySpeed = vMetersPerSecond * Math.sin(desiredDriveAngle) / 2;
    tardriangle = desiredDriveAngle;
    rot = Math.abs(omegaRadiansPerSecond);
    desiredDistance = targetDistance;
    v = vMetersPerSecond;
    targetHeading = TargetHeading;
    m_driveTrain = driveTrain;
    m_timer = new Timer();
    time = Math.abs(targetDistance / vMetersPerSecond);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrain);

    SmartDashboard.putNumber("time", time);
    SmartDashboard.putNumber("YSPEED", ySpeed);
    SmartDashboard.putNumber("XSPEED", xSpeed);
    SmartDashboard.putNumber("TOTAL SPEED", Math.hypot(xSpeed, ySpeed));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveTrain.brakeAll();
    m_driveTrain.setOrientation(true);

    m_timer.reset();
    m_timer.start();
    m_driveTrain.setInRange(false);

    rot *= targetHeading < m_driveTrain.getHeading() ? -1 : 1;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_timer.get() >= time) {
      xSpeed = 0;
      ySpeed = 0;
    }

    SmartDashboard.putNumber("TIMER", m_timer.get());
    SmartDashboard.putNumber("TURNERROR", targetHeading - m_driveTrain.getHeading());

    if (Math.abs(targetHeading - m_driveTrain.getHeading()) <= 2) {
      targetHeading = m_driveTrain.getHeading();
      rot = 0;

      xSpeed = v * Math.cos(tardriangle);
      ySpeed = v * Math.sin(tardriangle);
    }

    m_driveTrain.autonDrive(xSpeed, ySpeed, rot);
    m_driveTrain.setInRange(m_timer.get() <= (desiredDistance - AutoAimConstants.inRangeThreshold) / v);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.stop();
    m_driveTrain.lockPose();

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
