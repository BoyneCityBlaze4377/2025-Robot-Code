package frc.robot.commands.Auton.Functions;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

public class CurvedAutonDrive extends Command {
  private final DriveTrain m_driveTrain;
  private double x, y, xFactor, yFactor, slope, rot, turnError, relativeAngle, initialTurnValue, targetDegrees, limit, angle;
  private final Timer m_timer;

  /**
   * Drive the robot autonomously, but in a curve.
   * 
   * @param driveTrain The DriveTrain to drive.
   * @param Angle The desired angle of the curve.
   * @param scale Determines how large the curve is. Must be a positive number, 1 is default.
   * @param TargetDegrees the number of degrees to turn the robot by.
   * @param omega How fast to turn the robot. Must be between 0 and 1.
   * @param timeOut The maximum amount of time, in seconds, this can take.
   */
  public CurvedAutonDrive(DriveTrain driveTrain, double Angle, double scale, double TargetDegrees, double omega, double timeOut) {
    m_driveTrain = driveTrain;
    angle = Angle;
    targetDegrees = TargetDegrees;
    rot = omega;
    limit = timeOut;
    m_timer = new Timer();
    xFactor = .001 * scale;
    x = scale;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrain);

    angle = MathUtil.angleModulus(angle);
    // if (angle > 180) {
    //   angle -= 360;
    // } else if (angle < -180) {
    //   angle += 360;
    // }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_driveTrain.setOrientation(true);
    m_driveTrain.brakeAll();

    Math.copySign(x, angle);
    y = 0;

    slope = Math.tan(Units.degreesToRadians(angle));

    yFactor = xFactor / slope;

    initialTurnValue = m_driveTrain.getHeading();
    Math.copySign(targetDegrees, rot);
    rot *= 2;
    targetDegrees *= -1;

    m_timer.reset();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (y >= 1 || y <= -1) {
      m_driveTrain.teleopDrive(0, 0, rot);
    } else {
      m_driveTrain.teleopDrive(-x, y, rot);
      if (angle > 0 && angle < 90) {
        x -= xFactor;
        y += yFactor;
      } else if (angle > 90 && angle < 180) {
        x -= xFactor;
        y -= yFactor;
      } else if (angle < 0 && angle > -90) {
        x += xFactor;
        y += xFactor;
      } else if (angle < -90 && angle > -180) {
        x += xFactor;
        y -= yFactor;
      }
    }


    relativeAngle = m_driveTrain.getHeading() - initialTurnValue;
    turnError = targetDegrees + Math.copySign(relativeAngle, rot);
    // if (rot < 0) {
    //   turnError = targetDegrees - relativeAngle;
    // } else if (rot > 0) {
    //   turnError = targetDegrees + relativeAngle;
    // }

    if (Math.abs(turnError) <= 5) {
      targetDegrees = 0;
      rot = 0;
      turnError = 0;
    }

    SmartDashboard.putNumber("CRVX", x);
    SmartDashboard.putNumber("CRVY", y);
    SmartDashboard.putNumber("CurvedTurnError", turnError);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.get() >= limit || slope == Float.NaN || slope == 0 || (y < -1 || y > 1 && Math.abs(turnError) <= 5);
  }
}
