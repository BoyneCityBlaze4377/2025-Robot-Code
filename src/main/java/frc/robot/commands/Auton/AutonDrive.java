package frc.robot.commands.Auton;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.DriveTrain;

public class AutonDrive extends Command {
  private double xSpeed, ySpeed, rot, targetDegrees, turnError, relativeAngle, initialTurnValue, time;
  private boolean finished, driven, turned;
  private final DriveTrain m_driveTrain;
  private final Timer m_timer;
  /** Creates a new AutonDrive. */
  public AutonDrive(DriveTrain driveTrain, double x, double y, double omega, double targetDistance, double TargetDegrees) {
    xSpeed = x;
    ySpeed = y;
    rot = omega;
    targetDegrees = TargetDegrees;
    m_driveTrain = driveTrain;
    m_timer = new Timer();
    time = targetDistance / Math.hypot(x * SwerveConstants.maxSpeed * m_driveTrain.getSpeedScaler(), 
                                       y * SwerveConstants.maxSpeed * m_driveTrain.getSpeedScaler());
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrain);
    m_driveTrain.brakeAll();
    finished = false;
    driven = false;
    turned = false;
    m_driveTrain.setOrientation(true);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialTurnValue = m_driveTrain.getHeading();
    m_driveTrain.brakeAll();
    m_driveTrain.setOrientation(true);
    
    finished = false;
    driven = false;
    turned = false;

    Math.copySign(targetDegrees, rot);
    rot *= 2;
    targetDegrees *= -1;

    m_timer.reset();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    relativeAngle = m_driveTrain.getHeading() - initialTurnValue;
    turnError = targetDegrees + Math.copySign(relativeAngle, rot);

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
      targetDegrees = 0;
      rot = 0;
      turnError = 0;
    }

    m_driveTrain.autonDrive(-xSpeed, ySpeed, rot);

    SmartDashboard.putBoolean("driven", driven);
    SmartDashboard.putBoolean("turned", turned);
    SmartDashboard.putBoolean("autonDriveFinished", finished);
    SmartDashboard.putNumber("turnError", turnError);
    SmartDashboard.putNumber("relativeAngle", relativeAngle);
    SmartDashboard.putNumber("Timer", m_timer.get());
    SmartDashboard.putNumber("TargetDegrees", targetDegrees);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.stop();
    m_driveTrain.lockPose();

    targetDegrees = 0;
    rot = 0;
    xSpeed = 0;
    ySpeed = 0;

    m_timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.get() >= time && Math.abs(turnError) <= 5;
  }
}
