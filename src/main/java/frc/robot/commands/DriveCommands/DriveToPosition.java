package frc.robot.commands.DriveCommands;
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
  private double x, y, rot, outputLimit;
  private final Alignment m_alignment;

  /** Creates a new DriveToPosition. */
  public DriveToPosition(DriveTrain driveTrain, AdvancedPose2D DesiredPosition, Alignment alignment) {
    m_driveTrain = driveTrain;
    desiredPosition = DesiredPosition;

    xController = new PIDController(AutoAimConstants.transkP, AutoAimConstants.transkI, AutoAimConstants.transkD);
    xController.setTolerance(AutoAimConstants.transkTolerance);
    yController = new PIDController(AutoAimConstants.transkP, AutoAimConstants.transkI, AutoAimConstants.transkD);
    yController.setTolerance(AutoAimConstants.transkTolerance);
    turningController = new PIDController(AutoAimConstants.turnkP, AutoAimConstants.turnkI, AutoAimConstants.turnkD);
    turningController.setTolerance(AutoAimConstants.turnkTolerance);

    outputLimit = .6;

    m_alignment = alignment;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrain);
  }

  public DriveToPosition(DriveTrain driveTrain, int AprilTagID, Alignment alignment) {
    this(driveTrain, AutoAimConstants.reefStationPoseFromAprilTagID.get(AprilTagID), alignment);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch (m_alignment) {
      case left:
        desiredPosition.withRobotRelativeTransformation(new Translation2d(AutoAimConstants.leftCoralReefOffset, 0));
      break;
      case center:
        desiredPosition.withRobotRelativeTransformation(new Translation2d(0, 0));
      break;
      case right:
        desiredPosition.withRobotRelativeTransformation(new Translation2d(AutoAimConstants.rightCoralReefOffset, 0));
      break;
      default:
        desiredPosition.withRobotRelativeTransformation(new Translation2d(0, 0));
      break;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rot = MathUtil.clamp(turningController.calculate(m_driveTrain.getHeading(), 
                                                     desiredPosition.getRotation().getDegrees()), 
                         -outputLimit, outputLimit);

    x = MathUtil.clamp(xController.calculate(m_driveTrain.getPoseEstimate().getX(), 
                                             desiredPosition.getX()), 
                       -outputLimit, outputLimit);

    y = MathUtil.clamp(yController.calculate(m_driveTrain.getPoseEstimate().getY(), 
                                             desiredPosition.getY()), 
                       -outputLimit, outputLimit);

    if (Math.abs(x) < yController.getErrorTolerance()) x = 0;
    if (Math.abs(y) < xController.getErrorTolerance()) y = 0;
    if (Math.abs(rot) < turningController.getErrorTolerance()) rot = 0;

    m_driveTrain.autonDrive(x, y, rot);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.lockPose();
    x = 0;
    y = 0;
    rot = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return xController.atSetpoint() && yController.atSetpoint() && turningController.atSetpoint();
  }
}
