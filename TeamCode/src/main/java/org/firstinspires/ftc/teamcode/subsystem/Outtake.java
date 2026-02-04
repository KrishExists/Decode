package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
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

@Config
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
    public static double kP = TeamConstants.SHOOTER_kP;
    public static double kI = TeamConstants.SHOOTER_kI;
    public static double kD = TeamConstants.SHOOTER_kD;

    private double integral = TeamConstants.integral;
    private double lastError = TeamConstants.lastError;
    private double lastTime;
private PIDFController pidfController;
    private final ElapsedTime timer = new ElapsedTime();

    public Outtake(HardwareMap hw, Telemetry t) {
        this.hardwareMap = hw;
        this.telemetry = t;
        pidfController = new PIDFController(0.05,0,0,0.00024);

        // Map motors
        outtake = hardwareMap.get(DcMotorEx.class, "Outtake");
        outtake2 = hardwareMap.get(DcMotorEx.class, "Outtake2");

        // Encoder + direction setup
        outtake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtake2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtake2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        outtake.setDirection(DcMotorSimple.Direction.FORWARD);
        outtake2.setDirection(DcMotorSimple.Direction.REVERSE);

        // Linkage servo
        linkage = hardwareMap.get(Servo.class, "Linkage");

        lastTime = timer.seconds();
    }

    // =======================
    //  Shooter RPM Functions
    // =======================

    public double getRPM() {
        // identical to original Shooter logic
        return outtake.getVelocity() * 2.14;
    }

    public void spinToRpm(double targetRPM) {
        double currRPM = getRPM();

        // PID feedback
        double pidPower = pidfController.calculate(currRPM, targetRPM);

        // Feedforward

        // Combine and clip
        double power = Range.clip(pidPower , 0, 1);
        outtake.setPower(power);

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
    public void init() {
        setLinkage(TeamConstants.LINKAGE_REST);
        stop();
    }

    @Override
    public void update(Gamepad gamepad21, Gamepad gamepad2) {
        // no-op (Shooter controlled by other subsystems)
    }

    public void setPower(double p) {
        outtake.setPower(p);
        outtake2.setPower(p);
    }

    public double getVelocity() {
        return outtake.getVelocity();
    }

    public void setVelocity(int i) {
        outtake.setVelocity(i);
    }

    public void resetTime(){
        timer.reset();
    }
}
