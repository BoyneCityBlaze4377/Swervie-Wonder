package frc.Lib;

import edu.wpi.first.math.MathUtil;

/** Add your docs here. */
public class SCurveProfile {
    private SCurveConstraints m_constraints;
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

        public SCurveState(double position) {
            this(position, 0, 0);
        }
    }

    public SCurveProfile(SCurveConstraints constraints) {
        m_constraints = constraints;
    }

    public void setConstraints(SCurveConstraints constraints) {
        m_constraints = constraints;
    }

    public SCurveState calculate(double time, SCurveState currentState, SCurveState goalState) {
        direction = currentState.position > goalState.position ? -1 : 1;
        currentState = configureState(currentState);

        // Phase durations
        double Tj = m_constraints.maxAcceleration / m_constraints.maxJerk; // Time to ramp acceleration from 0 to aMax
        double Ta = Tj * 2;      // Total accel time without cruise
        double Td = Ta;          // Symmetrical decel
        double Tc;               // Cruise time
        double targetDistance = Math.abs(goalState.position - currentState.position);

        // Distance covered in acceleration and deceleration
        double accelerationDistance = m_constraints.maxAcceleration * Tj * Tj + m_constraints.maxAcceleration * (Ta - 2 * Tj) * Tj;
        double decelerationDistance = accelerationDistance; // symmetric
        double cruiseDistance = targetDistance - (accelerationDistance + decelerationDistance); // Remaining for cruise

        if (cruiseDistance < 0) {
            // No cruise possible, adjust acceleration times to fit within distance
            // Solve for Ta using quartic approximation
            Ta = Math.cbrt((6 * targetDistance) / m_constraints.maxJerk); // crude estimation
            Tj = Ta / 2;
            Tc = 0;
        } else {
            Tc = cruiseDistance / m_constraints.maxVelocity;
        }

        double t = 0.0, a = 0.0, v = 0.0, x = 0.0;
        MathUtil.clamp(time, 0, Ta + Tj + Tc + Td);

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
            else if (t < (Ta + Tc)) {
                jerk = 0;
                a = 0;
            }

            // Phase 5: jerk -j
            else if (t < (Ta + Tc + Tj)) {
                jerk = -m_constraints.maxJerk;
            }

            // Phase 6: jerk 0
            else if (t < (Ta + Tc + Td - Tj)) {
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

        return configureState(new SCurveState(x, v, a));
    }

    public SCurveState calculate(double time, double currentState, double goalState) {
        return calculate(time, new SCurveState(currentState), new SCurveState(goalState));
    }

    public SCurveState configureState(SCurveState in) {
        double position = in.position * direction;
        double velocity = MathUtil.clamp(in.velocity * direction, -m_constraints.maxVelocity, m_constraints.maxVelocity);
        double acceleration = MathUtil.clamp(in.acceleration * direction, -m_constraints.maxAcceleration, m_constraints.maxAcceleration);

        return new SCurveState(position, velocity, acceleration);
    }
}
