package frc.robot.commands.Auton.Functions;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

public class RobotRelativeAutonDrive extends Command {
  private double xSpeed, ySpeed, rot, targetHeading, turnError, relativeAngle, initialTurnValue, time;
  private final DriveTrain m_driveTrain;
  private final Timer m_timer;
  /** Creates a new AutonDrive. */
  public RobotRelativeAutonDrive(DriveTrain driveTrain, double desiredDriveAngle, double vMetersPerSecond, 
                    double omegaRadiansPerSecond, double targetDistance, double TargetHeading) {
    ySpeed = vMetersPerSecond * Math.cos(desiredDriveAngle);
    xSpeed = vMetersPerSecond * Math.sin(desiredDriveAngle);
    rot = omegaRadiansPerSecond;
    m_driveTrain = driveTrain;
    targetHeading = TargetHeading == 360 ? m_driveTrain.getHeading() : TargetHeading;
    m_timer = new Timer();
    time = targetDistance / Math.hypot(xSpeed, ySpeed);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialTurnValue = m_driveTrain.getHeading();
    m_driveTrain.brakeAll();
    m_driveTrain.setOrientation(false);

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

    MathUtil.inputModulus(relativeAngle, -360, 360);
    MathUtil.inputModulus(turnError, -360, 360);

    // if (relativeAngle >= 360) {
    //   relativeAngle -= 360;
    // }
   
    // if (relativeAngle <= -360) {
    //   relativeAngle += 360;
    // }

    // if (turnError >= 360) {
    //   turnError -= 360;
    // }
   
    // if (turnError <= -360) {
    //   turnError += 360;
    // } 

    if (m_timer.get() >= time) {
      xSpeed = 0;
      ySpeed = 0;
    }

    if (Math.abs(turnError) <= 5) {
      targetHeading = m_driveTrain.getHeading();
      rot = 0;
      turnError = 0;
    }

    m_driveTrain.autonDrive(xSpeed, ySpeed, rot);
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
