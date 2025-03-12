package frc.robot.commands.Auton.Functions;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AffectorConstants;
import frc.robot.Constants.AutoAimConstants;
import frc.robot.Constants.AutoAimConstants.Position;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.CoralAffector;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class InRangeAllToPosition extends Command {
  private final Elevator m_elevator;
  private final DriveTrain m_driveTrain;
  private final CoralAffector m_coralAffector;
  private final PIDController wristController, elevatorController;
  private final double elevatorTarget, coralWristTarget;
  private double elevatorOutput, coralWristOutput;
  
  public InRangeAllToPosition(Elevator elevator, CoralAffector coralAffector, DriveTrain driveTrain,
                            double elevatorPos, double wristPos) {
    m_elevator = elevator;
    m_coralAffector = coralAffector;
    m_driveTrain = driveTrain;
                          
    elevatorController = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);
    elevatorController.setTolerance(ElevatorConstants.kTolerance);
                          
    wristController = new PIDController(AffectorConstants.coralWristKP, 
                                        AffectorConstants.coralWristKI, 
                                        AffectorConstants.coralWristKD);
    wristController.setTolerance(AffectorConstants.coralWristKTolerance);
                          
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
    elevatorOutput = 0;
    coralWristOutput = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevatorOutput = MathUtil.clamp(elevatorController.calculate(m_elevator.getEncoderVal(), elevatorTarget), 
                                   ElevatorConstants.maxDownSpeed, ElevatorConstants.maxUpSpeed);
    coralWristOutput = MathUtil.clamp(wristController.calculate(m_coralAffector.getWristDegrees(), coralWristTarget), 
                                   AffectorConstants.maxCoralWristDownSpeed, AffectorConstants.maxCoralWristUpSpeed);
    
    if (m_driveTrain.getInRange()) {
      m_elevator.set(elevatorOutput);
      m_coralAffector.moveWrist(coralWristOutput);
    } else {
      m_elevator.lockElevator();
      m_coralAffector.PIDLockWrist();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevator.lockElevator();
    m_coralAffector.PIDLockWrist();

    elevatorOutput = 0;
    coralWristOutput = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return wristController.atSetpoint() && elevatorController.atSetpoint();
  }
}
