package frc.Lib.Terms;

import frc.Lib.Term;

/** Add your docs here. */
public class Pow extends Term {
    private final double m_exp;
    private final Term m_imbedded;

    public Pow(Term coefficient, Term imbedded, double exponent) {
        super(TermType.pow, coefficient, imbedded);
        m_exp = exponent;
        m_imbedded = imbedded;
    }

    public Pow(double coefficient, Term imbedded, double exponent) {
        this(new Constant(coefficient), imbedded, exponent);
    }

    @Override
    public double eval(double x) {
        return Math.pow(m_imbedded.eval(x), m_exp);
    }
}
