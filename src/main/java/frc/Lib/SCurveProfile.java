package frc.Lib;

/** Add your docs here. */
public class SCurveProfile {
    private final SCurveConstraints m_constraints;
    private SCurveState m_currentState;

    private int direction;

    public class SCurveConstraints {
        public final double maxVelocity;
        public final double maxAcceleration;
        public final double maxJerk;
        public final double period;

        public SCurveConstraints(double maxVelocity, double maxAcceleration, double maxJerk, double period) {
            this.maxVelocity = maxVelocity;
            this.maxAcceleration = maxAcceleration;
            this.maxJerk = maxJerk;
            this.period = period;
        }
    }

    public class SCurveState {
        public final double position;
        public final double velocity;
        public final double acceleration;

        public SCurveState(double position, double velocity, double acceleration) {
            this.position = position;
            this.velocity = velocity;
            this.acceleration = acceleration;
        }
    }

    public SCurveProfile(SCurveConstraints constraints) {
        m_constraints = constraints;
    }

    public SCurveState calculate(double time, SCurveState currentState, SCurveState goalState) {
        // Phase durations
        double Tj = m_constraints.maxAcceleration / m_constraints.maxJerk; // Time to ramp acceleration from 0 to aMax
        double Ta = Tj * 2;      // Total accel time without cruise
        double Td = Ta;          // Symmetrical decel
        double Tv;               // Cruise time
        double targetDistance = Math.abs(goalState.position - currentState.position);

        // Distance covered in acceleration and deceleration
        double Sa = m_constraints.maxAcceleration * Tj * Tj + m_constraints.maxAcceleration * (Ta - 2 * Tj) * Tj;
        double Sd = Sa; // symmetric
        double Sc = targetDistance - (Sa + Sd); // Remaining for cruise

        if (Sc < 0) {
            // No cruise possible, adjust acceleration times to fit within distance
            // Solve for Ta using quartic approximation
            Ta = Math.cbrt((6 * targetDistance) / m_constraints.maxJerk); // crude estimation
            Tj = Ta / 2;
            Tv = 0;
        } else {
            Tv = Sc / m_constraints.maxVelocity;
        }

        double t = 0.0, a = 0.0, v = 0.0, x = 0.0;

        while (t < time) {
            double jerk;

            // Phase 1: jerk +j
            if (t < Tj) {
                jerk = m_constraints.maxJerk;
            }

            // Phase 2: jerk 0
            else if (t < (Ta - Tj)) {
                jerk = 0;
            }

            // Phase 3: jerk -j
            else if (t < Ta) {
                jerk = -m_constraints.maxJerk;
            }

            // Phase 4: cruise
            else if (t < (Ta + Tv)) {
                jerk = 0;
                a = 0;
            }

            // Phase 5: jerk -j
            else if (t < (Ta + Tv + Tj)) {
                jerk = -m_constraints.maxJerk;
            }

            // Phase 6: jerk 0
            else if (t < (Ta + Tv + Td - Tj)) {
                jerk = 0;
            }

            // Phase 7: jerk +j
            else {
                jerk = m_constraints.maxJerk;
            }

            // Integrate
            a += jerk * m_constraints.period;
            v += a * m_constraints.period;
            x += v * m_constraints.period;
            t += m_constraints.period;
        }

        return new SCurveState(x, v, a);
    }

    public static void simulateProfile(double Tj, double Ta, double Tv, double Td, double jMax) {
        double dt = 0.01;
        double t = 0.0;
        double a = 0.0, v = 0.0, x = 0.0;
        double totalTime = Ta + Tv + Td;

        while (t <= totalTime) {
            double jerk;

            // Phase 1: jerk +j
            if (t < Tj) {
                jerk = jMax;
            }
            // Phase 2: jerk 0
            else if (t < (Ta - Tj)) {
                jerk = 0;
            }
            // Phase 3: jerk -j
            else if (t < Ta) {
                jerk = -jMax;
            }
            // Phase 4: cruise
            else if (t < (Ta + Tv)) {
                jerk = 0;
                a = 0;
            }
            // Phase 5: jerk -j
            else if (t < (Ta + Tv + Tj)) {
                jerk = -jMax;
            }
            // Phase 6: jerk 0
            else if (t < (Ta + Tv + Td - Tj)) {
                jerk = 0;
            }
            // Phase 7: jerk +j
            else {
                jerk = jMax;
            }

            // Integrate
            a += jerk * dt;
            v += a * dt;
            x += v * dt;

            System.out.printf("t=%.2f s | x=%.4f m | v=%.4f m/s | a=%.4f m/s²%n", t, x, v, a);
            t += dt;
        }
    }
}
