package frc.Lib.Terms;

import frc.Lib.Term;

/** Add your docs here. */
public class Constant extends Term {
    private final double m_value;
    public Constant(double value) {
        super(TermType.Constant, null, null);
        m_value = value;
    }

    @Override
    public double evaluate(double x) {
        return m_value;
    }
}
