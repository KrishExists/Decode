package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.sfdev.assembly.state.StateMachine;

@TeleOp(name = "MecanumTeleopMain", group = "Main")
public class MainTele extends LinearOpMode {

    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    private DcMotor intake;
    private DcMotor outtake;

    @Override
    public void runOpMode() throws InterruptedException {
        // ====== Initialize Motors ======
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        intake = hardwareMap.get(DcMotor.class, "Intake");
        outtake = hardwareMap.get(DcMotor.class, "Outtake");

        // ====== Motor Config ======
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reverse the left side (common for mecanum)
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // ====== Initialize DecodeController ======
        DecodeController decodeController = new DecodeController(hardwareMap);
        decodeController.controllerInit(); // ✅ ensures proper initial state setup
        StateMachine shooterMachine = decodeController.shooterMachine(gamepad1, gamepad2);

        telemetry.addLine("Initialized — Waiting for start");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // ====== Main Control Loop ======
        while (opModeIsActive()) {
            // Update the shooter/intake state machine
            shooterMachine.update();

            // Mecanum drive control
            double y = -gamepad1.left_stick_y; // forward/back
            double x = gamepad1.left_stick_x * 1.1; // strafe, with small correction
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

            // Optional telemetry feedback
            telemetry.addData("State", decodeController.state);
            telemetry.addData("Front Left", frontLeftPower);
            telemetry.addData("Front Right", frontRightPower);
            telemetry.addData("Back Left", backLeftPower);
            telemetry.addData("Back Right", backRightPower);
            telemetry.update();
        }
    }
}
