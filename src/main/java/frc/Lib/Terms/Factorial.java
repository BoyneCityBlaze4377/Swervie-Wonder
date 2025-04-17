package frc.Lib.Terms;

import frc.Lib.FactorialUtil;
import frc.Lib.Term;

/** Add your docs here. */
public class Factorial extends Term {
    public Factorial(Term coefficient, Term imbedded) {
        super(TermType.factorial, coefficient, imbedded);
    }

    public Factorial(double coefficient, Term imbedded) {
        this(new Constant(coefficient), imbedded);
    }

    public Factorial(Term imbedded) {
        this(NoCoefficient, imbedded);
    }

    @Override
    public double eval(double x) {
        return FactorialUtil.factorial((int) m_imbedded.eval(x));
    }
}
