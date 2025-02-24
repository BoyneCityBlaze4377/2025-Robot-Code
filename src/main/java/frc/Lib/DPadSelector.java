package frc.Lib;

/** Add your docs here. */
public class DPadSelector<OutputType> {
    private final int[] m_degrees;

    /* 0 is up, degrees increase clockwise, no input is -1 */
    public DPadSelector(int... degrees) {
        m_degrees = degrees;
    }

    public OutputType selectOutput(int degrees, OutputType defaultOutput, OutputType... outputs) {
        OutputType selectedOutput = defaultOutput;
        for (int i = 0; i < outputs.length; i++) {
            if (degrees == m_degrees[i]) selectedOutput = outputs[i];
        }
        return selectedOutput;
    }

    public int[] getDegreesArray() {
        return m_degrees;
    }
}
