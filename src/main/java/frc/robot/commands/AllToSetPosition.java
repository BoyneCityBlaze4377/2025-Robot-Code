package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AffectorConstants;
import frc.robot.Constants.AutoAimConstants;
import frc.robot.Constants.AutoAimConstants.Position;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.CoralAffector;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AllToSetPosition extends Command {
  private final Elevator m_elevator;
  private final CoralAffector m_coralAffector;
  private final PIDController wristController, elevatorController;
  private final double elevatorTarget, coralWristTarget;
  private double elevatorOutput, coralWristOutput;
  private final Position position;
  
  /** Creates a new SelectPosition. */
  public AllToSetPosition(Elevator elevator, CoralAffector coralAffector, Position desiredPosition) {
    m_elevator = elevator;
    m_coralAffector = coralAffector;
    position = desiredPosition;

    elevatorController = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);
    elevatorController.setTolerance(ElevatorConstants.kTolerance);

    wristController = new PIDController(AffectorConstants.coralWristKP, 
                                        AffectorConstants.coralWristKI, 
                                        AffectorConstants.coralWristKD);
    wristController.setTolerance(AffectorConstants.coralWristKTolerance);

    elevatorTarget = AutoAimConstants.positionValues.get(position)[0];
    coralWristTarget = AutoAimConstants.positionValues.get(position)[1];
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_elevator, m_coralAffector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevatorOutput = 0;
    coralWristOutput = 0;

    m_elevator.setPositionString("Going to " + position.toString());
    m_elevator.setAtPos(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevatorOutput = MathUtil.clamp(elevatorController.calculate(m_elevator.getEncoderVal(), elevatorTarget), 
                                    ElevatorConstants.maxDownSpeed, ElevatorConstants.maxUpSpeed);
    m_elevator.set(elevatorOutput);

    coralWristOutput = MathUtil.clamp(wristController.calculate(m_coralAffector.getWristDegrees(), coralWristTarget), 
                                      AffectorConstants.maxCoralWristDownSpeed, AffectorConstants.maxCoralWristUpSpeed);
    m_coralAffector.moveWrist(coralWristOutput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevator.lockElevator();
    m_coralAffector.lockWrist();

    elevatorOutput = 0;
    coralWristOutput = 0;

    m_elevator.setPositionString("At " + position.toString());
    m_elevator.setAtPos(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return wristController.atSetpoint() && elevatorController.atSetpoint();
  }
}
