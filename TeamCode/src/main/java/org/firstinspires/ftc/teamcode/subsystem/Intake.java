package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake implements Subsystem {
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    public DcMotor intake;

    public Intake(HardwareMap h, Telemetry t) {
        this.hardwareMap = h;
        this.telemetry = t;

        intake = hardwareMap.get(DcMotor.class, "Intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void setPower(double power) {
        intake.setPower(power);
    }

    @Override
    public void init() {

    }

    @Override
    public void update() {
    }
}
