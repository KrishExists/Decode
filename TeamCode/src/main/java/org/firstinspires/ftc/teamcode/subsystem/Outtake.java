package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.TeamConstants;

public class Outtake implements Subsystem {

    // =======================
    //  Hardware
    // =======================
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;

    public DcMotorEx outtake;
    public DcMotorEx outtake2;
    public Servo linkage;


    // Shooter PID variables (from Shooter class)

    private PIDFController pidfController;
    private final ElapsedTime timer = new ElapsedTime();

    public Outtake(HardwareMap hw, Telemetry t) {
        this.hardwareMap = hw;
        this.telemetry = t;
        pidfController = new PIDFController(0.005,0,0,0.000215);
        pidfController.setTolerance(Double.POSITIVE_INFINITY, 10);
        // Map motors
        outtake = hardwareMap.get(DcMotorEx.class, "Outtake2");
        outtake2 = hardwareMap.get(DcMotorEx.class, "Outtake");

        // Encoder + direction setup
        outtake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtake2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtake2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        outtake.setDirection(DcMotorSimple.Direction.FORWARD);
        outtake2.setDirection(DcMotorSimple.Direction.FORWARD);

        // Linkage servo
        linkage = hardwareMap.get(Servo.class, "Linkage");

    }

    // =======================
    //  Shooter RPM Functions
    // =======================

    public double getRPM() {
        // identical to original Shooter logic
        return Math.abs(outtake.getVelocity() * 2.14);
    }

    public double getRPM2() {
        return Math.abs(outtake2.getVelocity()*2.14);
    }

    public void spinToRpm(double targetRPM) {
    double currRPM = getRPM();

    // PID feedback
    double pidPower = pidfController.calculate(currRPM, targetRPM);


    // Combine and clip
    double power = Range.clip(pidPower , 0, 1);

    double currRPM2 = getRPM2();

    // PID feedback
    double pidPower2 = pidfController.calculate(currRPM2, targetRPM);

    // Feedforward

    // Combine and clip
    double power2 = Range.clip(pidPower2, 0, 1);
        outtake.setPower(power);

        outtake2.setPower(power2);


}

    public boolean atSpeed(double low, double high) {
        double rpm = getRPM();
        return rpm >= low && rpm <= high;
    }

    public void stop() {
        outtake.setPower(TeamConstants.outtake_Stop);
        outtake2.setPower(TeamConstants.outtake_Stop);
    }


    // =======================
    // Linkage control
    // =======================
    public void setLinkage(double pos) {
        linkage.setPosition(pos);
    }



    @Override
    public void update(Gamepad gamepad21, Gamepad gamepad2) {
        // no-op (Shooter controlled by other subsystems)
    }

    public void setPower(double p) {
        outtake.setPower(p);
        outtake2.setPower(p);
    }



}
