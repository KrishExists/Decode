package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "EncoderAuto_WithIntakeSync", group = "Main")
public class EncodersAutoDecode extends LinearOpMode {

    // ====== Drive Motors ======
    private DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;

    // ====== Mechanism Motors ======
    private DcMotor intake;
    private DcMotorEx outtake;
    private Servo linkage;

    // ====== Constants ======
    static final double COUNTS_PER_MOTOR_REV = 537.7; // goBilda 5202 motor
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    static final double COUNTS_PER_INCH =
            (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

    private final ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        // ====== Hardware Init ======
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");

        intake = hardwareMap.get(DcMotor.class, "Intake");
        outtake = hardwareMap.get(DcMotorEx.class, "Outtake");
        linkage = hardwareMap.get(Servo.class, "Linkage");

        // Directions and Behaviors
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        for (DcMotor m : new DcMotor[]{frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linkage.setPosition(0.8);

        resetEncoders();

        telemetry.addLine("Autonomous Initialized — Waiting for start");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // Example autonomous sequence with synced intake/outtake
           // runIntake(true);               // start intake
            moveForward(1);               // move forward 12 inches while intaking
//            runIntake(false);              // stop intake
//
//            runOuttakeSequence();          // perform full outtake sequence
//
//            strafeRight(8);
//            turnRight(90);
//            moveBackward(10);
        }
    }

    // ====== Movement Methods ======

    public void moveForward(double inches) {
        encoderDrive(0.5, inches, inches, inches, inches);
    }

    public void moveBackward(double inches) {
        encoderDrive(0.5, -inches, -inches, -inches, -inches);
    }

    public void strafeRight(double inches) {
        encoderDrive(0.5, inches, -inches, -inches, inches);
    }

    public void strafeLeft(double inches) {
        encoderDrive(0.5, -inches, inches, inches, -inches);
    }

    public void turnRight(double degrees) {
        double inchesPerDegree = 0.1;
        double inches = degrees * inchesPerDegree;
        encoderDrive(0.5, inches, -inches, inches, -inches);
    }

    public void turnLeft(double degrees) {
        double inchesPerDegree = 0.1;
        double inches = degrees * inchesPerDegree;
        encoderDrive(0.5, -inches, inches, -inches, inches);
    }

    // ====== Encoder Drive Core ======

    public void encoderDrive(double speed,
                             double frontLeftInches, double frontRightInches,
                             double backLeftInches, double backRightInches) {

        int newFrontLeftTarget = frontLeftMotor.getCurrentPosition() + (int) (frontLeftInches * COUNTS_PER_INCH);
        int newFrontRightTarget = frontRightMotor.getCurrentPosition() + (int) (frontRightInches * COUNTS_PER_INCH);
        int newBackLeftTarget = backLeftMotor.getCurrentPosition() + (int) (backLeftInches * COUNTS_PER_INCH);
        int newBackRightTarget = backRightMotor.getCurrentPosition() + (int) (backRightInches * COUNTS_PER_INCH);

        // Set targets
        frontLeftMotor.setTargetPosition(newFrontLeftTarget);
        frontRightMotor.setTargetPosition(newFrontRightTarget);
        backLeftMotor.setTargetPosition(newBackLeftTarget);
        backRightMotor.setTargetPosition(newBackRightTarget);

        // Set mode
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Power on
        frontLeftMotor.setPower(speed);
        frontRightMotor.setPower(speed);
        backLeftMotor.setPower(speed);
        backRightMotor.setPower(speed);

        // Wait for completion
        while (opModeIsActive() &&
                (frontLeftMotor.isBusy() || frontRightMotor.isBusy() ||
                        backLeftMotor.isBusy() || backRightMotor.isBusy())) {
            telemetry.addData("FL Target", newFrontLeftTarget);
            telemetry.addData("FR Target", newFrontRightTarget);
            telemetry.addData("BL Target", newBackLeftTarget);
            telemetry.addData("BR Target", newBackRightTarget);
            telemetry.update();
        }

        stopAllMotors();
        resetRunMode();
    }

    // ====== Mechanism Control ======

    public void runIntake(boolean enable) {
        if (enable) {
            intake.setPower(0.8);
            outtake.setPower(-0.2);
            linkage.setPosition(0.8);
        } else {
            intake.setPower(0);
            outtake.setPower(0);
        }
    }

    public void runOuttakeSequence() {
        timer.reset();
        while (opModeIsActive() && timer.milliseconds() < 4000) {
            double t = timer.milliseconds();

            if (t < 700) {
                outtake.setPower(-0.8);
                intake.setPower(-0.5);
                linkage.setPosition(0.8);
            } else if (t < 3500) {
                outtake.setPower(1.0);
                intake.setPower(0.0);
                linkage.setPosition(0.8);
            } else {
                outtake.setVelocity(6000);
                linkage.setPosition(0.6);
                intake.setPower(0.5);
            }
            telemetry.addData("Outtake Timer", t);
            telemetry.update();
        }
        intake.setPower(0);
        outtake.setPower(0);
        linkage.setPosition(0.8);
    }

    // ====== Utility Methods ======

    public void resetEncoders() {
        for (DcMotor m : new DcMotor[]{frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor}) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void stopAllMotors() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }

    public void resetRunMode() {
        for (DcMotor m : new DcMotor[]{frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor}) {
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}
