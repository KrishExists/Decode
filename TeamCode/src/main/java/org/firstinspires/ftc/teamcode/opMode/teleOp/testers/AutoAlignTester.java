package org.firstinspires.ftc.teamcode.opMode.teleOp.testers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

@Config
@TeleOp(name="Auto Align", group="TeleOp")
public class AutoAlignTester extends LinearOpMode {

    // Motors
    DcMotor frontLeft, frontRight, backLeft, backRight;

    // IMU
    IMU imu;

    // PID values (tune these)
    public static double kP = 0.01;
    public static double kI = 0.0;
    public static double kD = 0.001;

    double integral = 0;
    double lastError = 0;
    ElapsedTime pidTimer = new ElapsedTime();

    // Target heading (degrees)
    public static double targetHeading = 0.0;

    @Override
    public void runOpMode() {

        // ---------------- Motors ----------------
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRight = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRight  = hardwareMap.get(DcMotor.class, "backRightMotor");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // ---------------- IMU ----------------
        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );

        imu.initialize(parameters);

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        pidTimer.reset();

        while (opModeIsActive()) {

            // ---------------- Gamepad ----------------
            double y = -gamepad1.left_stick_y;   // forward/back
            double x = gamepad1.left_stick_x;    // strafe
            double rx = gamepad1.right_stick_x;  // manual turn

            // ---------------- Auto Align Buttons ----------------
            if (gamepad1.a) targetHeading = 0;     // Face forward
            if (gamepad1.b) targetHeading = 90;    // Face right
            if (gamepad1.x) targetHeading = -90;   // Face left
            if (gamepad1.y) targetHeading = 180;   // Face backward

            boolean autoAlign = gamepad1.a || gamepad1.b || gamepad1.x || gamepad1.y;

            double turnCorrection = 0;

            if (autoAlign) {
                turnCorrection = headingPID(targetHeading);
            } else {
                // reset PID when not aligning
                integral = 0;
                lastError = 0;
                pidTimer.reset();
            }

            // ---------------- Mecanum Math ----------------
            double fl = y + x + rx + turnCorrection;
            double bl = y - x + rx + turnCorrection;
            double fr = y - x - rx - turnCorrection;
            double br = y + x - rx - turnCorrection;

            // Normalize
            double max = Math.max(1.0,
                    Math.max(Math.abs(fl),
                            Math.max(Math.abs(bl),
                                    Math.max(Math.abs(fr), Math.abs(br)))));

            frontLeft.setPower(fl / max);
            backLeft.setPower(bl / max);
            frontRight.setPower(fr / max);
            backRight.setPower(br / max);

            // ---------------- Telemetry ----------------
            telemetry.addData("Heading", getHeading());
            telemetry.addData("Target", targetHeading);
            telemetry.addData("Error", angleError(targetHeading, getHeading()));
            telemetry.update();
        }
    }

    // ================= IMU METHODS =================

    double getHeading() {
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        return angles.getYaw(AngleUnit.DEGREES);
    }

    double angleError(double target, double current) {
        double error = target - current;
        while (error > 180) error -= 360;
        while (error < -180) error += 360;
        return error;
    }

    double headingPID(double target) {
        double current = getHeading();
        double error = angleError(target, current);

        double dt = pidTimer.seconds();
        pidTimer.reset();

        integral += error * dt;
        double derivative = (error - lastError) / dt;
        lastError = error;

        return (kP * error) + (kI * integral) + (kD * derivative);
    }
}
