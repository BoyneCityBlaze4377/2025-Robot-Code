package frc.Lib;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;

/** Add your docs here. */
public class DPadSelector {

    private final Joystick m_operatorStick;

    private final Command m_command1, m_command2, m_command3, m_command4, m_noSelectCommand;
    private Command selectedCommand;
    private final int m_degree1, m_degree2, m_degree3, m_degree4;

    public DPadSelector(Joystick operatorStick, int degree1, int degree2, int degree3, int degree4,
                        Command command1, Command command2, Command command3, Command command4, Command noSelectCommand) {
        m_operatorStick = operatorStick;

        m_command1 = command1;
        m_command2 = command2;
        m_command3 = command3;
        m_command4 = command4;

        m_noSelectCommand = noSelectCommand;

        m_degree1 = degree1;
        m_degree2 = degree2;
        m_degree3 = degree3;
        m_degree4 = degree4;
    }

    public Command selectCommand() {
        if (m_operatorStick.getPOV() == m_degree1) {
            selectedCommand = m_command1;
        } else if (m_operatorStick.getPOV() == m_degree2) {
            selectedCommand = m_command2;
        } else if (m_operatorStick.getPOV() == m_degree3) {
            selectedCommand = m_command3;
        } else if (m_operatorStick.getPOV() == m_degree4) {
            selectedCommand = m_command4;
        } else {
            selectedCommand = m_noSelectCommand;
        }

        return selectedCommand;
    }
}
