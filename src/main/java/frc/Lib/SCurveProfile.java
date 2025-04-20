package frc.Lib;

import java.util.ArrayList;

import edu.wpi.first.math.MathUtil;

/** Add your docs here. */
public class SCurveProfile {
    private SCurveConstraints m_constraints;

    private SCurveState m_startingState, m_goalState, m_lastState;

    private double Tj, Ta, Td, Tc, totalTime, 
                   targetDistance, accelerationDistance, decelerationDistance, cruiseDistance,
                   phase1Time, phase2Time, phase3Time, phase4Time, phase5Time, phase6Time;

    private int direction;

    private static final double sigChangePosThreshold = 0;
    private static final double sigChangeVelThreshold = 0;
    private static final double sigChangeAccelThreshold = 0;

    public static class SCurveConstraints {
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

    public static class SCurveState {
        public double position;
        public double velocity;
        public double acceleration;

        public SCurveState(double position, double velocity, double acceleration) {
            this.position = position;
            this.velocity = velocity;
            this.acceleration = acceleration;
        }

        public SCurveState(double position) {
            this(position, 0, 0);
        }

        public SCurveState () {
            this(0, 0, 0);
        }
    }

    public static class TimedSCurveState extends SCurveState{
        public double time;
        public TimedSCurveState(double position, double velocity, double acceleration, double time) {
            super(position, velocity, acceleration);
            this.time = time;
        }

        public TimedSCurveState(double position, double velocity, double acceleration) {
            this(position, velocity, acceleration, 0);
        }

        public TimedSCurveState(double position) {
            this(position, 0, 0, 0);
        }
    }

    public SCurveProfile(SCurveConstraints constraints, SCurveState startingState, SCurveState goalState) {
        m_constraints = constraints;
        m_startingState = startingState;
        m_goalState = goalState;
        configureProfile();
    }

    public SCurveProfile (SCurveConstraints constraints) {
        this(constraints, new SCurveState(), new SCurveState());
    }

    public void setConstraints(SCurveConstraints constraints) {
        m_constraints = constraints;
    }

    public void setStartState(SCurveState startState) {
        m_startingState = configureState(startState);
        configureProfile();
    }

    public void setGoalState(SCurveState goalState) {
        m_goalState = configureState(goalState);
        configureProfile();
    }

    public void setGoalState(double position, double velocity, double acceleration) {
        m_goalState = new SCurveState(position, velocity, acceleration);
    }

    public void setGoalState(double position) {
        m_goalState.position = position;
    }

    private void setCurrentstate(SCurveState currentState) {
        if (isSignificantStateChange(currentState)) {
            setStartState(currentState);
        }
        m_lastState = currentState;
    }

    public SCurveState getGoalState() {
        return m_goalState;
    }

    public double getTotalTime() {
        return totalTime;
    }

    public void configureProfile() {
        direction = (int) Math.signum(m_goalState.position - m_startingState.position);

        Tj = m_constraints.maxAcceleration / m_constraints.maxJerk;

        // Compute acceleration time
        if ((m_constraints.maxVelocity - m_startingState.velocity) < m_constraints.maxAcceleration * Tj) {
            Ta = 2 * Math.sqrt((m_constraints.maxVelocity - m_startingState.velocity) / m_constraints.maxJerk);
        } else {
            double Tconst = (m_constraints.maxVelocity - m_startingState.velocity - m_constraints.maxAcceleration * Tj) / m_constraints.maxAcceleration;
            Ta = 2 * Tj + Tconst;
        }

        // Compute deceleration time
        if (Math.abs((m_constraints.maxVelocity - m_goalState.velocity)) < m_constraints.maxAcceleration * Tj) {
            Td = 2 * Math.sqrt((m_constraints.maxVelocity - m_goalState.velocity) / m_constraints.maxJerk);
        } else {
            double Tconst = (m_constraints.maxVelocity - m_goalState.velocity - m_constraints.maxAcceleration * Tj) / m_constraints.maxAcceleration;
            Td = 2 * Tj + Tconst;
        }

        // Distances
        accelerationDistance = (m_startingState.velocity + m_constraints.maxVelocity) / 2.0 * Ta;
        decelerationDistance = (m_constraints.maxVelocity + m_goalState.velocity) / 2.0 * Td;
        targetDistance = Math.abs(m_goalState.position - m_startingState.position);
        cruiseDistance = targetDistance - accelerationDistance - decelerationDistance;

        // Cruise Time
        Tc = (cruiseDistance > 0) ? cruiseDistance / m_constraints.maxVelocity : 0;

        totalTime = Ta + Tc + Td;

        // Time at the end of each phase of the curve
        phase1Time = Tj;
        phase2Time = Ta - Tj;
        phase3Time = Ta;
        phase4Time = Ta + Tc;
        phase5Time = phase4Time + Tj;
        phase6Time = phase4Time + Td - Tj;

        m_lastState = m_startingState;
    }

    public SCurveState calculate(double time, SCurveState currentState) {
        double t = 0.0, 
               a = m_startingState.acceleration, 
               v = m_startingState.velocity, 
               x = m_startingState.position;
        time = MathUtil.clamp(time, 0, totalTime);

        while (t < time) {
            double jerk;

            if (t < phase1Time) {
                jerk = m_constraints.maxJerk;
            } else if (t < phase2Time) {
                jerk = 0;
            } else if (t < phase3Time) {
                jerk = -m_constraints.maxJerk;
            } else if (t < phase4Time) {
                jerk = 0;
            } else if (t < phase5Time) {
                jerk = -m_constraints.maxJerk;
            } else if (t < phase6Time) {
                jerk = 0;
            } else {
                jerk = m_constraints.maxJerk;
            }

            // Integrate
            a += jerk * m_constraints.period * direction;
            v += a * m_constraints.period * direction;
            x += v * m_constraints.period * direction;

            t += m_constraints.period;
        }

        if (Math.abs(time - (Ta + Tc + Td)) <= 1e-6) {
            v = m_goalState.velocity;
            a = m_goalState.acceleration;
            x = m_goalState.position;
        }

        setCurrentstate(currentState);
        return new SCurveState(x, v, a);
    }

    public ArrayList<TimedSCurveState> getProfAsList(double time) {
        ArrayList<TimedSCurveState> prof = new ArrayList<TimedSCurveState>();
        double t = 0.0, 
               a = m_startingState.acceleration, 
               v = m_startingState.velocity, 
               x = m_startingState.position;
        time = MathUtil.clamp(time, 0, totalTime);

        while (t < time) {
            double jerk;

            if (t < phase1Time) {
                jerk = m_constraints.maxJerk;
            } else if (t < phase2Time) {
                jerk = 0;
            } else if (t < phase3Time) {
                jerk = -m_constraints.maxJerk;
            } else if (t < phase4Time) {
                jerk = 0;
                a = 0;
            } else if (t < phase5Time) {
                jerk = -m_constraints.maxJerk;
            } else if (t < phase6Time) {
                jerk = 0;
            } else {
                jerk = m_constraints.maxJerk;
            }

            // Integrate
            a += jerk * m_constraints.period * direction;
            v += a * m_constraints.period * direction;
            x += v * m_constraints.period * direction;

            prof.add(new TimedSCurveState(x, v, a, t));

            t += m_constraints.period;
        }

        if (Math.abs(time - (Ta + Tc + Td)) <= 1e-6) {
            v = m_goalState.velocity;
            a = m_goalState.acceleration;
            x = m_goalState.position;
        }

        return prof;
    }

    public static boolean isSignificantStateChange(SCurveState state1, SCurveState state2) {
        return (Math.abs(state1.position - state2.position) > sigChangePosThreshold ||
                Math.abs(state1.velocity - state2.velocity) > sigChangeVelThreshold ||
                Math.abs(state1.acceleration - state2.acceleration) > sigChangeAccelThreshold);
    }

    public static boolean isSignificantStateChange(SCurveState state1, SCurveState state2,
                                                   double positionThreshold, double velocityThreshold,
                                                   double accelerationThreshold) {
        return (Math.abs(state1.position - state2.position) > positionThreshold ||
                Math.abs(state1.velocity - state2.velocity) > velocityThreshold ||
                Math.abs(state1.acceleration - state2.acceleration) > accelerationThreshold);
    }

    public boolean isSignificantStateChange(SCurveState currentState) {
        return isSignificantStateChange(currentState, m_lastState);
    }

    public SCurveState configureState(SCurveState in) {
        double position = in.position;
        double velocity = MathUtil.clamp(in.velocity, -m_constraints.maxVelocity, m_constraints.maxVelocity);
        double acceleration = MathUtil.clamp(in.acceleration, -m_constraints.maxAcceleration, m_constraints.maxAcceleration);

        return new SCurveState(position, velocity, acceleration);
    }
}
