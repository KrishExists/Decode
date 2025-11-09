package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Outtake Test w/ Timer", group = "")
public class Outtaketesttwo extends LinearOpMode {

    private DcMotorEx outtake;
    private Servo linkage;
    FtcDashboard dashboard;

    private PIDController rpmPID;
    private double kp = 0.00625;
    private double ki = 0;
    private double kd = 0.001;

    private static final double rpmThresh = 50;
    private final ElapsedTime timer = new ElapsedTime();
    private double timeToReachRPM = 0;
    private boolean reachedOnce = false;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize hardware

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        dashboard = FtcDashboard.getInstance();

        outtake = hardwareMap.get(DcMotorEx.class, "Outtake");
        linkage = hardwareMap.get(Servo.class, "Linkage");

        outtake.setDirection(DcMotorEx.Direction.REVERSE);
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rpmPID = new PIDController(kp, ki, kd);
        linkage.setPosition(0.92); // default init position

        telemetry.addData("Status", "Initialized — press Play");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        timer.reset(); // start timing after start is pressed

        while (opModeIsActive()) {

            // Spin up to 5000 rpm target
            spinToRpm(5000);

            if (upToRpm(5000) && !reachedOnce) {
                timeToReachRPM = timer.milliseconds();
                reachedOnce = true;
            }

            // Telemetry display
            telemetry.addData("Time to reach RPM (ms)", timeToReachRPM);
            telemetry.addData("Velocity (ticks/s)", getVelocity());
            telemetry.addData("At target RPM?", upToRpm(5000));
            telemetry.addData("Current RPM", currentRPM());
            telemetry.addData("Motor Power", outtake.getPower());
            telemetry.update();
        }
    }

    // --- Helper methods ---
    private void spinToRpm(double rpm) {
        rpmPID.setPID(kp, ki, kd);
        double output = rpmPID.calculate(currentRPM(), rpm);
        output = Range.clip(output, 0, 1);
        outtake.setPower(output);
    }

    private boolean upToRpm(double rpm) {
        double curr = currentRPM();
        return curr > rpm - rpmThresh && curr < rpm + rpmThresh;
    }

    private double currentRPM() {
        return outtake.getVelocity() * 2.2;
    }

    private double getVelocity() {
        return outtake.getVelocity();
    }
}
