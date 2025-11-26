package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.util.Constants;


public class Shooter {


    private final DcMotorEx outtake, outtake2;


    private double integral = 0;
    private double lastError = 0;
    private double lastTime = 0;


    public Shooter(HardwareMap hw) {
        outtake = hw.get(DcMotorEx.class, "Outtake");
        outtake2 = hw.get(DcMotorEx.class, "Outtake2");


        outtake.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        outtake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        outtake2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        outtake2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


        lastTime = System.nanoTime() / 1e9;
    }


    private double getRPM() {
        return outtake.getVelocity() * 2.2;
    }


    public void spin(double targetRPM) {
        double current = getRPM();
        double error = targetRPM - current;


        double now = System.nanoTime() / 1e9;
        double dt = now - lastTime;


        integral += error * dt;
        double derivative = (error - lastError) / dt;


        double output = Constants.SHOOTER_kP * error
                + Constants.SHOOTER_kI * integral
                + Constants.SHOOTER_kD * derivative;


        output = Range.clip(output, 0, 1);


        outtake.setPower(output);
        outtake2.setPower(output);


        lastError = error;
        lastTime = now;
    }


    public boolean atSpeed(double low, double high) {
        double rpm = getRPM();
        return rpm >= low && rpm <= high;
    }


    public void stop() {
        outtake.setPower(0);
        outtake2.setPower(0);
    }


    public void reverse() {
        outtake.setPower(-0.8);
        outtake2.setPower(-0.8);
    }
}