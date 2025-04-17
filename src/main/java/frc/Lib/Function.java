package frc.Lib;

/** Add your docs here. */
public class Function {
    public final Term[] m_terms;

    public Function(Term... terms) {
        m_terms = terms;
    }

    public Term[] getTerms() {
        return m_terms;
    }

    public double evaluate(double x) {
        double value = 0;
        for (int i = 0; i < m_terms.length; i++) {
            value += m_terms[i].evaluate(x);
        }

        return value;
    }

    public double approxIntegral(double minX, double maxX, double intervalLength) {
        double integral = 0;
        for (double x = minX; x <= maxX; x += intervalLength) {
            integral += evaluate(x);
        }
        return integral;
    }
}
