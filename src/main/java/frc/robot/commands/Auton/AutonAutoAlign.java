package frc.robot.commands.Auton;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.Lib.DPadSelector;
import frc.robot.Constants.AutoAimConstants;
import frc.robot.Constants.AutoAimConstants.Alignment;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.VisionSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutonAutoAlign extends Command {
  private final DriveTrain m_driveTrain;
  private double xSpeed, ySpeed, rot;
  private double targetDistance, targetOffsetDeg, targetAngle, maxHorizOutput, maxDisOutput, maxRotOutput, pov;
  private final PIDController angleController, horizController, distanceController;
  private final Alignment alignment;
  private final VisionSubsystem m_visionSubsystem;

  /** Creates a new LimeLightDrive. */
  public AutonAutoAlign(DriveTrain driveTrain, VisionSubsystem visionSubsystem, Joystick stick, double TargetDistance, Alignment a) {
    m_driveTrain = driveTrain;
    m_visionSubsystem = visionSubsystem;

    pov = stick.getPOV();

    targetDistance = TargetDistance;
    alignment = a;

    horizController = new PIDController(.1, 0, 0);
    distanceController = new PIDController(.01, 0, 0);
    angleController = new PIDController(.15, 0, 0);

    horizController.setTolerance(1);
    distanceController.setTolerance(20);
    angleController.setTolerance(2);

    addRequirements(m_driveTrain);

    maxHorizOutput = .8;
    maxDisOutput = .3;
    maxRotOutput = .5;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("RUNNING", true);
    SmartDashboard.putNumber("targetD", targetDistance);
    //SmartDashboard.putNumber("POV", pov);

    targetOffsetDeg = AutoAimConstants.offsetFromAlignment.get(alignment) == 0 ? AutoAimConstants.LLDefaultOffsetDegrees : 
                      1 / Math.atan(AutoAimConstants.offsetFromAlignment.get(alignment) / targetDistance) 
                      + AutoAimConstants.LLDefaultOffsetDegrees;
    targetAngle = 0;
    //AutoAimConstants.angleFromReefStation.get(AutoAimConstants.reefStationFromAprilTagID.get(
    //                                                         m_visionSubsystem.getTargetID()));
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

    m_driveTrain.teleopDrive(-xSpeed, 0, -0);

    SmartDashboard.putNumber("PIDOUT", distanceController.calculate(m_visionSubsystem.getDistanceMeasurementmm(), targetDistance * 1000));
    SmartDashboard.putNumber("XSPEED", xSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.autonDrive(0, 0, 0);
    xSpeed = 0;
    ySpeed = 0;
    rot = 0;
    SmartDashboard.putBoolean("RUNNING", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (horizController.atSetpoint() && distanceController.atSetpoint() && angleController.atSetpoint());
            //|| m_visionSubsystem.getTargetID() == 0;
  }
}
