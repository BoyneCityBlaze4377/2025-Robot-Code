package frc.Lib.Terms;

import frc.Lib.Term;

/** Add your docs here. */
public class Pow extends Term {
    private final double m_exp;
    private final Term m_imbedded;

    public Pow(Term coefficient, Term imbedded, double exponent) {
        super(TermType.Pow, coefficient, imbedded);
        m_exp = exponent;
        m_imbedded = imbedded;
    }

    @Override
    public double evaluate(double x) {
        return super.evaluate(Math.pow(m_imbedded.evaluate(x), m_exp));
    }
}
