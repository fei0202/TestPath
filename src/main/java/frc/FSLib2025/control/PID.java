package frc.FSLib2025.control;

public class PID {
    private double kp, ki, kd;
    private double i = 0, iLimit, iWindup;
    private double lastError = 0;
    
    public PID(double kp, double ki, double kd, double iLimit, double iWindup) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.iLimit = iLimit;
        this.iWindup = iWindup;
    }

    public PID(double kp, double ki, double kd) {
        this(kp, ki, kd, 0, 0);
    }

    public PID(double kp, double kd) {
        this(kp, 0, kd, 0, 0);
    }

    public double calculate(double error) {
        double pOut = kp * error;
        i += Math.abs(error) < iWindup ? error : 0;
        i = i > iLimit ? iLimit : i;
        i = i < -iLimit ? -iLimit : i;
        double iOut = ki * i;
        double dOut = kd * (error - lastError);
        lastError = error;
        return pOut + iOut + dOut;
    }
}
