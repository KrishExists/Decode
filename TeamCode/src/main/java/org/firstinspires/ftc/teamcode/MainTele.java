package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "MecanumTeleopMain_Switch", group = "Main")
public class MainTele extends LinearOpMode {

    // ====== Hardware ======
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;

    private DcMotor intake;
    private DcMotor outtake;
    private Servo linkage;

    // ====== Timer ======
    private final ElapsedTime timer = new ElapsedTime();

    // ====== Intake System State ======
    private enum IntakeState {
        INTAKE,
        OUTTAKE,
        REST
    }

    private IntakeState currentState = IntakeState.REST;
    private IntakeState previousState = IntakeState.REST;

    @Override
    public void runOpMode() throws InterruptedException {
        // ====== Initialize Motors ======
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");

        intake = hardwareMap.get(DcMotor.class, "Intake");
        outtake = hardwareMap.get(DcMotor.class, "Outtake");
        linkage = hardwareMap.get(Servo.class, "Linkage");

        // ====== Motor Config ======
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        linkage.setPosition(0.0);

        telemetry.addLine("Initialized — Waiting for start");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // ====== MAIN LOOP ======
        while (opModeIsActive()) {

            // ====== Determine Intake State ======
            if (gamepad2.right_trigger > 0.2) {
                currentState = IntakeState.INTAKE;
            } else if (gamepad2.left_trigger > 0.2) {
                currentState = IntakeState.OUTTAKE;
            } else {
                currentState = IntakeState.REST;
            }

            // ====== Detect state change ======
            if (currentState != previousState) {
                timer.reset(); // reset timer only once when switching states
                previousState = currentState;
            }

            // ====== Handle Intake State ======
            switch (currentState) {
                case INTAKE:
                    intake.setPower(0.8);
                    outtake.setPower(-0.2);
                    linkage.setPosition(0.0);
                    break;

                case OUTTAKE:
                    if (timer.milliseconds() < 750) {
                        // First 750ms — run outtake motor
                        outtake.setPower(1.0);
                        intake.setPower(0.0);
                        linkage.setPosition(0.0);
                    } else {
                        // After 750ms — stop outtake, maybe reverse intake
                        outtake.setPower(1.0);
                        intake.setPower(0.8); // optional: reverse briefly
                        linkage.setPosition(0.5);
                    }
                    break;

                case REST:
                default:
                    intake.setPower(0.0);
                    outtake.setPower(0.0);
                    linkage.setPosition(0.0);
                    break;
            }

            // ====== Mecanum Drive ======
            double y = -gamepad1.left_stick_y; // forward/back
            double x = gamepad1.left_stick_x * 1.1; // strafe (tuned)
            double rx = gamepad1.right_stick_x; // rotation

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            // ====== Telemetry ======
            telemetry.addData("State", currentState);
            telemetry.addData("Timer (ms)", timer.milliseconds());
            telemetry.addData("Intake Power", intake.getPower());
            telemetry.addData("Outtake Power", outtake.getPower());
            telemetry.addData("Linkage Pos", linkage.getPosition());
            telemetry.update();
        }
    }
}
