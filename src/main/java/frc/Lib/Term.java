package frc.Lib;

/** Add your docs here. */
public class Term {
    public enum TermType {Pow, LogBase, Exp, Constant, Sin, Cos, Tan, Sec, Csc, Cot, aSin, aCos, aTan, aSec, aCsc, aCot, Sqrt, var};

    public static TermType termType;
    private Term m_coefficient, m_imbedded;

    public Term(TermType type, Term coefficient, Term imbedded) {
        termType = type;
        m_coefficient = coefficient;
        m_imbedded = imbedded;
    }

    public double evaluate(double x) { 
        return m_coefficient.evaluate(x) * this.evaluate(x);
    }
}
