package frc.robot.commands.PieceAffectorsCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AffectorConstants;
import frc.robot.subsystems.CoralAffector;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralWristToPos extends Command {
  private final CoralAffector m_affector;
  private final double desiredPos;
  private final PIDController wristController;

  /** Creates a new ElevatorToPosition. */
  public CoralWristToPos(CoralAffector affector, double DesiredPos) {
    m_affector = affector;
    desiredPos = DesiredPos;
    wristController = new PIDController(AffectorConstants.coralWristKP, 
                                        AffectorConstants.coralWristKI, 
                                        AffectorConstants.coralWristKD);
    wristController.setTolerance(AffectorConstants.coralWristKTolerance);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_affector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = MathUtil.clamp(wristController.calculate(m_affector.getWristDegrees(), desiredPos), 
                                   AffectorConstants.maxCoralWristDownSpeed, AffectorConstants.maxCoralWristDownSpeed);
    m_affector.moveWrist(-output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_affector.lockWrist();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return wristController.atSetpoint();
  }
}
