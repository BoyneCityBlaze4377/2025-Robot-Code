package frc.Lib;

import frc.Lib.Terms.Constant;
import frc.Lib.Terms.Variable;

/** Add your docs here. */
public class Term {
    public enum TermType {pow, logBase, ln, exp, constant, sin, cos, tan, sec, csc, cot, 
                          aSin, aCos, aTan, aSec, aCsc, aCot, root, factorial, var};

    public static TermType termType;
    public static Term m_coefficient, m_imbedded;

    public static final Term NoCoefficient = new Constant(1);
    public static final Term Variable = new Variable();

    public Term(TermType type, Term coefficient, Term imbedded) {
        termType = type;
        m_coefficient = coefficient;
        m_imbedded = imbedded;
    }

    public native double eval(double x);

    public double evaluate(double x) {
        return m_coefficient.eval(x) * this.eval(x);
    };
}
