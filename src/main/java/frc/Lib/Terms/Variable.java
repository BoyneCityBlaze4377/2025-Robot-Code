package frc.Lib.Terms;

import frc.Lib.Term;

/** Add your docs here. */
public class Variable extends Term {
    public Variable(Term coefficient) {
        super(TermType.var, coefficient, null);
    }

    @Override
    public double evaluate(double x) {
        return x;
    }
}
