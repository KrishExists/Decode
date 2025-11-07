package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name = "Encoders Auton")
public class Encoders extends LinearOpMode {

    RobotHardware robot = new RobotHardware();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();

        robot.startEncoder();
        robot.encoderDriveBackwardInches(32);
        turnLeft(1, 300);
        sleep(250);
        robot.encoderDriveBackwardInches(10);
        robot.outtake1();
        sleep(10000);
        robot.rest();
        robot.encoderDriveForwardInches(15);
        turnLeft(1, 100);
        sleep(100);
        robot.encoderDriveForwardInches(6);
        robot.encoderDriveRightInches(10);
        // Example: Run intake and outtake sequence autonomously
//        robot.intake();
//        sleep(2000);
//        robot.outtake1();
//        robot.rest();
    }

    private void turnLeft(double power, long duration) {
        setMotorPowers(-power, power, -power, power);
        sleep(duration);
        stopMotors();
    }

    private void turnRight(double power, long duration) {
        setMotorPowers(power, -power, power, -power);
        sleep(duration);
        stopMotors();
    }

    private void setMotorPowers(double frontLeft, double frontRight, double backLeft, double backRight) {
        robot.frontLeftMotor.setPower(frontLeft);
        robot.frontRightMotor.setPower(frontRight);
        robot.backLeftMotor.setPower(backLeft);
        robot.backRightMotor.setPower(backRight);
    }

    private void stopMotors() {
        setMotorPowers(0, 0, 0, 0);
    }

    public class RobotHardware {
        // ===== Drive Motors =====
        private DcMotor frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;

        // ===== Intake/Outtake Hardware =====
        private DcMotor intake;
        private DcMotorEx outtake;
        private Servo linkage;

        private PIDController rpmPID;
        private final ElapsedTime timer = new ElapsedTime();

        public final double COUNTS_PER_MM = 537.5;

        public void init(HardwareMap hardwareMap) {
            // Drive Motors
            frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
            frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
            backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
            backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");

            // Intake/Outtake
            intake = hardwareMap.get(DcMotor.class, "Intake");
            outtake = hardwareMap.get(DcMotorEx.class, "Outtake");
            linkage = hardwareMap.get(Servo.class, "Linkage");

            // Motor Directions
            frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
            backRightMotor.setDirection(DcMotor.Direction.REVERSE);
            frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
            backLeftMotor.setDirection(DcMotor.Direction.FORWARD);

            // Motor Behaviors
            frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            outtake.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            outtake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

            linkage.setPosition(0.8);

            // PID setup (requires ShooterConstants class)
            rpmPID = new PIDController(ShooterConstants.kp, ShooterConstants.ki, ShooterConstants.kd);
        }

        // ===== Encoder Control =====
        public void startEncoder() {
            frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        public void stopEncoder() {
            frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        public void encoderDriveForwardInches(double inches) throws InterruptedException {
            double TotalTicks = inches * COUNTS_PER_MM * 25.4;

            frontLeftMotor.setTargetPosition((int) TotalTicks);
            frontRightMotor.setTargetPosition((int) TotalTicks);
            backRightMotor.setTargetPosition((int) TotalTicks);
            backLeftMotor.setTargetPosition((int) TotalTicks);
            setDrivePower(0.5);
            setRunToPosition();
            sleep((long) (inches * 25.4 * 2));
            resetEncoders();
        }

        public void encoderDriveBackwardInches(double inches) throws InterruptedException {
            double TotalTicks = inches * COUNTS_PER_MM * 25.4;

            frontLeftMotor.setTargetPosition(-(int) TotalTicks);
            backLeftMotor.setTargetPosition(-(int) TotalTicks);
            frontRightMotor.setTargetPosition(-(int) TotalTicks);
            backRightMotor.setTargetPosition(-(int) TotalTicks);
            setDrivePower(0.5);
            setRunToPosition();
            sleep((long) (inches * 25.4 * 2));
            resetEncoders();
        }

        public void encoderDriveLeftInches(double inches) throws InterruptedException {
            double TotalTicks = inches * COUNTS_PER_MM * 25.4;

            frontLeftMotor.setTargetPosition(-(int) TotalTicks);
            backLeftMotor.setTargetPosition((int) TotalTicks);
            frontRightMotor.setTargetPosition((int) TotalTicks);
            backRightMotor.setTargetPosition(-(int) TotalTicks);
            setDrivePower(0.5);
            setRunToPosition();
            sleep((long) (inches * 25.4 * 2));
            resetEncoders();
        }

        public void encoderDriveRightInches(double inches) throws InterruptedException {
            double TotalTicks = inches * COUNTS_PER_MM * 25.4;

            frontLeftMotor.setTargetPosition((int) TotalTicks);
            backLeftMotor.setTargetPosition(-(int) TotalTicks);
            frontRightMotor.setTargetPosition(-(int) TotalTicks);
            backRightMotor.setTargetPosition((int) TotalTicks);
            setDrivePower(0.5);
            setRunToPosition();
            sleep((long) (inches * 25.4 * 2));
            resetEncoders();
        }

        private void setDrivePower(double power) {
            frontLeftMotor.setPower(power);
            backLeftMotor.setPower(power);
            frontRightMotor.setPower(power);
            backRightMotor.setPower(power);
        }

        private void setRunToPosition() {
            frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        private void resetEncoders() {
            frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        // ===== Intake/Outtake Logic =====
        public void intake() {
            intake.setPower(0.8);
            outtake.setPower(-0.2);
            linkage.setPosition(0.8);
        }

        public void outtake1() throws InterruptedException {
            timer.reset();

            while (timer.milliseconds() < 2800 && opModeIsActive()) {
                outtake.setVelocity(1500);
                intake.setPower(0.0);
                linkage.setPosition(0.8);
            }

            while (timer.milliseconds() < 3300 && opModeIsActive()) {
                outtake.setVelocity(2400);
                linkage.setPosition(0.28);
                intake.setPower(0.0);
            }

            outtake.setVelocity(2400);
            linkage.setPosition(0.28);
            intake.setPower(0.8);
        }

        public void outtake() throws InterruptedException {
            timer.reset();

            while (timer.milliseconds() < 700 && opModeIsActive()) {
                outtake.setPower(-0.8);
                intake.setPower(-0.5);
                linkage.setPosition(0.8);
            }

            while (timer.milliseconds() < 3500 && opModeIsActive()) {
                outtake.setPower(1.0);
                intake.setPower(0.0);
                linkage.setPosition(0.8);
            }

            while (timer.milliseconds() < 4000 && opModeIsActive()) {
                outtake.setVelocity(2400);
                linkage.setPosition(0.28);
                intake.setPower(0.0);
            }

            outtake.setVelocity(2400);
            linkage.setPosition(0.28);
            intake.setPower(0.8);
        }

        public void runSlow() {
            intake.setPower(-1.0);
            outtake.setPower(-0.8);
            linkage.setPosition(0.8);
        }

        public void rest() {
            intake.setPower(0.0);
            outtake.setPower(0.0);
            linkage.setPosition(0.8);
        }
    }
}
