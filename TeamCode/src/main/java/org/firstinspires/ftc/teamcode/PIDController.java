package org.firstinspires.ftc.teamcode;

class PIDController {
    private double kp; // Proportional gain
    private double ki; // Integral gain
    private double kd;// Derivative gain
    private double kcos;
    private double prevTime;

    private double integral; // Integral sum
    private double previousError; // Previous error

    public PIDController(double kp, double ki, double kd, double kcos) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.integral = 0;
        this.previousError = 0;
        this.kcos = kcos;
        this.prevTime = System.nanoTime()/1e9;
    }

    public void setCoeffs (double kp, double ki, double kd, double kcos) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.kcos = kcos;
    }

    public double calculate(double setpoint, double current) {
        double error = setpoint - current;
        double time = System.nanoTime()/1e9;
        integral += error * ki*(time - prevTime); // Accumulate the integral
        double derivative = (error - previousError)/(time - prevTime); // Calculate the derivative
        double cos = Math.cos(Math.toRadians(360.0*(current)/2252.0)) * kcos;
        // Calculate PID output
        double output = (kp * error) + integral + (kd * derivative) + cos;
        // Store current error for next calculation
        previousError = error;
        prevTime = time;

        return output; // Return the calculated power
    }

    public void reset() {
        integral = 0;
        previousError = 0;
    }
}
