package frc.robot.commands.DriveCommands;

import java.util.HashMap;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.Lib.AdvancedPose2D;
import frc.robot.Constants.AutoAimConstants;
import frc.robot.Constants.AutoAimConstants.ReefStation;
import frc.robot.subsystems.DriveTrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TEMPAUTODRIVE extends Command {
  private final DriveTrain m_driveTrain;
  private ReefStation desiredStation;
  private AdvancedPose2D desiredPose;
  private final HashMap<ReefStation, AdvancedPose2D> reef;


  /** Creates a new AutoAimDrive. */
  public TEMPAUTODRIVE(DriveTrain driveTrain) {
    m_driveTrain = driveTrain;
    desiredStation = ReefStation.front;
    reef = m_driveTrain.getAlliance() == Alliance.Blue ? AutoAimConstants.blueReef : AutoAimConstants.redReef;
    desiredPose = reef.get(desiredStation);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    desiredStation = m_driveTrain.getDesiredStation();
    desiredPose = reef.get(desiredStation).withReefAlignment(m_driveTrain.getDesiredAlignment(), false);
    m_driveTrain.setPIDSetpoints(desiredPose.getX(),
                                 desiredPose.getY(),
                                 desiredPose.getRotation().getRadians());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    desiredPose = reef.get(desiredStation).withReefAlignment(m_driveTrain.getDesiredAlignment(), false);
    m_driveTrain.setPIDSetpoints(desiredPose.getX(),
                                 desiredPose.getY(),
                                 desiredPose.getRotation().getRadians());
    
    m_driveTrain.PIDDrive();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!interrupted) m_driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_driveTrain.atSetpoints();
  }
}
