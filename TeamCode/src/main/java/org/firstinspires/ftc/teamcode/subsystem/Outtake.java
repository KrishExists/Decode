package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public class Outtake implements Subsystem {
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    public DcMotorEx outtake;
    public Servo linkage;

    public PIDController rpmPID;
    public double kp = 0.00625;
    public double ki = 0;
    public double kd = 0.001;

    public static double rpmThresh = 50;

    public Outtake(HardwareMap h, Telemetry t) {
        this.hardwareMap = h;
        this.telemetry = t;

        outtake = hardwareMap.get(DcMotorEx.class, "Outtake");
        outtake.setDirection(DcMotorEx.Direction.REVERSE);
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtake.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        outtake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        linkage = hardwareMap.get(Servo.class, "Linkage");

        rpmPID = new PIDController(kp, ki, kd);
    }

    // -- outtake --
    public void spinToRpm(double rpm) {
        rpmPID.setPID(kp, ki, kd);
        double output = rpmPID.calculate(currentRPM(), rpm);
        output = Range.clip(output, 0, 1);
        outtake.setPower(output);
    }

    public boolean upToRpm(double rpm) {
        double curr = currentRPM();
        return curr > rpm - rpmThresh && curr < rpm + rpmThresh;
    }

    public double currentRPM() {
        double conversion = 60 / (2 * Math.PI);
        return outtake.getVelocity(AngleUnit.RADIANS) * conversion;
    }

    // -- linkage --
    public void setLinkage(double pos) {
        linkage.setPosition(pos);
    }

    @Override
    public void init() {
        setLinkage(0.92);
    }

    @Override
    public void update() {

    }
}
