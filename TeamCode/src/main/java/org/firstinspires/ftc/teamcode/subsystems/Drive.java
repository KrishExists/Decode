package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.Hardware;


public class Drive {
    private final Hardware hw;


    public Drive(Hardware hardware) {
        hw = hardware;
    }


    public void manualDrive(Gamepad g) {
        double y = -g.left_stick_y;
        double x = g.left_stick_x * 1.1;
        double rx = g.right_stick_x;


        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);


        hw.frontLeftMotor.setPower((y + x + rx) / denominator);
        hw.backLeftMotor.setPower((y - x + rx) / denominator);
        hw.frontRightMotor.setPower((y - x - rx) / denominator);
        hw.backRightMotor.setPower((y + x - rx) / denominator);
    }


    public void stop() {
        hw.frontLeftMotor.setPower(0);
        hw.backLeftMotor.setPower(0);
        hw.frontRightMotor.setPower(0);
        hw.backRightMotor.setPower(0);
    }

    public void setMotorPowers(double fl, double fr, double bl, double br) {
        hw.frontLeftMotor.setPower(fl);
        hw.frontRightMotor.setPower(fr);
        hw.backLeftMotor.setPower(bl);
        hw.backRightMotor.setPower(br);
    }

}