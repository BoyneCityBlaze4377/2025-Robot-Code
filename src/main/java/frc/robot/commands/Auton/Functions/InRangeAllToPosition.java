package frc.robot.commands.Auton.Functions;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoAimConstants;
import frc.robot.Constants.AutoAimConstants.Position;
import frc.robot.subsystems.CoralAffector;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class InRangeAllToPosition extends Command {
  private final Elevator m_elevator;
  private final DriveTrain m_driveTrain;
  private final CoralAffector m_coralAffector;
  private final double elevatorTarget, coralWristTarget;
  
  public InRangeAllToPosition(Elevator elevator, CoralAffector coralAffector, DriveTrain driveTrain,
                            double elevatorPos, double wristPos) {
    m_elevator = elevator;
    m_coralAffector = coralAffector;
    m_driveTrain = driveTrain;
                          
    elevatorTarget = elevatorPos;
    coralWristTarget = wristPos;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_elevator, m_coralAffector);
  }

  public InRangeAllToPosition(Elevator elevator, CoralAffector coralAffector, DriveTrain driveTrain, Position desiredPosition) {
    this(elevator, coralAffector, driveTrain, AutoAimConstants.positionValues.get(desiredPosition)[0], 
                                              AutoAimConstants.positionValues.get(desiredPosition)[1]);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elevator.setSetpoint(elevatorTarget);
    m_coralAffector.setSetpoint(coralWristTarget);
    m_coralAffector.setIsOverride(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {    
    if (m_driveTrain.getInRange()) {
      m_elevator.PIDMove();
      m_coralAffector.PIDMoveWrist();
    } else {
      m_elevator.lockElevator();
      m_coralAffector.lockWrist();
    }

    SmartDashboard.putBoolean("RANGE", m_driveTrain.getInRange());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevator.lockElevator();
    m_coralAffector.lockWrist();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_elevator.atSetpoint() && m_coralAffector.atSetpoint();
  }
}
