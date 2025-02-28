package frc.robot.commands.Auton.Functions;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoAimConstants;
import frc.robot.subsystems.DriveTrain;

public class AutonDrive extends Command {
  private double xSpeed, ySpeed, rot, targetHeading, turnError, relativeAngle, initialTurnValue, time, desiredDistance, v;
  private final DriveTrain m_driveTrain;
  private final Timer m_timer;
  /** Creates a new AutonDrive. */
  public AutonDrive(DriveTrain driveTrain, double desiredDriveAngle, double vMetersPerSecond, 
                    double omegaRadiansPerSecond, double targetDistance, double TargetHeading) {
    xSpeed = vMetersPerSecond * Math.cos(desiredDriveAngle);
    ySpeed = vMetersPerSecond * Math.sin(desiredDriveAngle);
    rot = omegaRadiansPerSecond;
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
    relativeAngle = m_driveTrain.getHeading() - initialTurnValue;
    turnError = targetHeading + Math.copySign(relativeAngle, rot);
    // relativeAngle = MathUtil.inputModulus(m_driveTrain.getHeading() - initialTurnValue, 
    //                                       -360, 360);
    // turnError = MathUtil.inputModulus(targetHeading + Math.copySign(relativeAngle, rot), 
    //                                   -360, 360);

    if (relativeAngle >= 360) {
      relativeAngle -= 360;
    }
   
    if (relativeAngle <= -360) {
      relativeAngle += 360;
    }

    if (turnError >= 360) {
      turnError -= 360;
    }
   
    if (turnError <= -360) {
      turnError += 360;
    } 

    if (m_timer.get() >= time) {
      xSpeed = 0;
      ySpeed = 0;
    }

    SmartDashboard.putNumber("TIMER", m_timer.get());

    if (Math.abs(turnError) <= 5) {
      targetHeading = m_driveTrain.getHeading();
      rot = 0;
      turnError = 0;
    }

    m_driveTrain.autonDrive(xSpeed, ySpeed, -rot);
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
    return m_timer.get() >= time && Math.abs(turnError) <= 2;
  }
}
