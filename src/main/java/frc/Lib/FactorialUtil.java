package frc.Lib;

/** Add your docs here. */
public class FactorialUtil {

    /**
     * Calculate the factorial of a given integer
     * 
     * @param k The interger to calculate the factorial of
     * @return The calculated factorial
     */
    public static double factorial(int k) {
        long m_k = 1;
        for (int i = 1; i <= k; i++) {
            m_k *= i;
        }
        return m_k;
    }
}
