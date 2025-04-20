package frc.Lib;

/** Add your docs here. */
public class TimedValue {
    public double value;
    public double time;

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

    public double getAverage(TimedValue other) {
        return (this.value + other.value) / 2;
    }

    public double getRateOfChange(TimedValue other) {
        return (this.value - other.value) / (this.time - other.time);
    }
}
