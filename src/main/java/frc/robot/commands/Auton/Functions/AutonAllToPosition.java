package frc.robot.commands.Auton.Functions;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoAimConstants;
import frc.robot.Constants.AutoAimConstants.Position;
import frc.robot.subsystems.CoralAffector;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutonAllToPosition extends Command {
  private final Elevator m_elevator;
  private final CoralAffector m_coralAffector;
  private final double elevatorTarget, coralWristTarget;
  
  public AutonAllToPosition(Elevator elevator, CoralAffector coralAffector, 
                            double elevatorPos, double wristPos) {
    m_elevator = elevator;
    m_coralAffector = coralAffector;
                          
    elevatorTarget = elevatorPos;
    coralWristTarget = wristPos;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_elevator, m_coralAffector);
  }

  public AutonAllToPosition(Elevator elevator, CoralAffector coralAffector, Position desiredPosition) {
    this(elevator, coralAffector, AutoAimConstants.positionValues.get(desiredPosition)[0], 
                                  AutoAimConstants.positionValues.get(desiredPosition)[1]);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elevator.setSetpoint(elevatorTarget);
    m_coralAffector.setSetpoint(coralWristTarget);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_elevator.PIDMove();
    m_coralAffector.PIDMoveWrist();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevator.lockElevator();
    m_coralAffector.PIDLockWrist();
}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_elevator.atSetpoint() && m_coralAffector.atSetpoint();
  }
}
