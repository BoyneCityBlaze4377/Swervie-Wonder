package frc.Lib;

/** Add your docs here. */
public class TimedValue {
    private double value;
    private double time;

    public TimedValue(double value, double time) {
        this.value = value;
        this.time = time;
    }

    public void setValue(double value) {
        this.value = value;
    }

    public void setTime(double time) {
        this.time = time;
    }

    public void setParams(double value, double time) {
        this.value = value;
        this.time = time;
    }

    public double getValue() {
        return this.value;
    }

    public double getTime() {
        return this.time;
    }

    public double getAverage(TimedValue other) {
        return (this.value + other.value) / 2;
    }

    public double getRateOfChange(TimedValue other) {
        return (this.value - other.value) / (this.time - other.time);
    }
}
