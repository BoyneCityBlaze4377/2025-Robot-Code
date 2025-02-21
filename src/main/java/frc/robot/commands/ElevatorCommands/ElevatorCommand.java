package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ElevatorCommand extends Command {
  private final Elevator m_elevator;
  private final Joystick m_stick;

  /** Creates a new ElevatorCommand. */
  public ElevatorCommand(Elevator elevator, Joystick stick) {
    m_elevator = elevator;
    m_stick = stick;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_stick.getRawAxis(2) > .95 && m_stick.getRawAxis(3) > .95) {
      m_elevator.set(-m_stick.getRawAxis(1) * .8);
    } else if (m_stick.getRawButton(6)) {
        m_elevator.up();
    } else if (m_stick.getRawButton(5)) {
        m_elevator.down();
    } else {
      m_elevator.lockElevator();
    }

    if (m_stick.getRawButton(10)) m_elevator.zeroEncoder();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevator.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
