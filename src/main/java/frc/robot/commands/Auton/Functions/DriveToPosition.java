package frc.robot.commands.Auton.Functions;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.Lib.AdvancedPose2D;
import frc.robot.subsystems.DriveTrain;
import frc.robot.Constants.AutoAimConstants;
import frc.robot.Constants.AutoAimConstants.Alignment;

public class DriveToPosition extends Command {
  private final DriveTrain m_driveTrain;
  private final AdvancedPose2D desiredPosition;
  private final PIDController xController, yController, turningController;
  private double x, y, rot;
  private final double maxXOutput, maxYOutput, maxRotOutput;

  /** Creates a new DriveToPosition. */
  public DriveToPosition(DriveTrain driveTrain, AdvancedPose2D DesiredPosition, Alignment alignment) {
    m_driveTrain = driveTrain;
    desiredPosition = DesiredPosition.withRobotRelativeTransformation(
                      new Translation2d(AutoAimConstants.offsetFromAlignment.get(alignment), 0));

    xController = new PIDController(AutoAimConstants.horizkP, AutoAimConstants.horizkI, AutoAimConstants.horizkD);
    xController.setTolerance(AutoAimConstants.horizkTolerance);

    yController = new PIDController(AutoAimConstants.horizkP, AutoAimConstants.horizkI, AutoAimConstants.horizkD);
    yController.setTolerance(AutoAimConstants.horizkTolerance);

    turningController = new PIDController(AutoAimConstants.turnkP, AutoAimConstants.turnkI, AutoAimConstants.turnkD);
    turningController.setTolerance(AutoAimConstants.turnkTolerance);

    maxXOutput = 3;
    maxYOutput = 3;
    maxRotOutput = Math.PI/2;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrain);
  }

  public DriveToPosition(DriveTrain driveTrain, int AprilTagID, Alignment alignment) {
    this(driveTrain, AutoAimConstants.reefStationPoseFromAprilTagID.get(AprilTagID), alignment);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xController.setSetpoint(desiredPosition.getX());
    yController.setSetpoint(desiredPosition.getY());
    turningController.setSetpoint(desiredPosition.getRotation().getDegrees());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    x = MathUtil.clamp(xController.calculate(m_driveTrain.getPoseEstimate().getX()), 
                            -maxXOutput, maxXOutput);
    y = MathUtil.clamp(yController.calculate(m_driveTrain.getPoseEstimate().getY()), 
                            -maxYOutput, maxYOutput);
    rot = MathUtil.clamp(turningController.calculate(m_driveTrain.getHeading()), 
                         -maxRotOutput, maxRotOutput);

    m_driveTrain.autonDrive(-x, y, -rot);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return xController.atSetpoint() && yController.atSetpoint() && turningController.atSetpoint();
  }
}
