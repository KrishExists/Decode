
package org.firstinspires.ftc.teamcode.opMode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * BASIC AUTONOMOUS TEMPLATE
 * Goal: score 18 balls total (example: 3 preload + 5 cycles of 3)
 * This is a LOGIC + STRUCTURE auto, not pathing-based.
 * You will plug in your real distances, sensors, and mechanisms.
 */

@Autonomous(name = "18 Ball Auto (Basic)", group = "Auto")
public class GPTSHT extends LinearOpMode {

    // Drive motors
    DcMotor leftFront, rightFront, leftRear, rightRear;

    // Mechanisms (rename to match config)
    DcMotor intake;
    DcMotor shooter;
    DcMotor transfer;

    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {

        // ===== Hardware Map =====
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftRear   = hardwareMap.get(DcMotor.class, "leftRear");
        rightRear  = hardwareMap.get(DcMotor.class, "rightRear");

        intake   = hardwareMap.get(DcMotor.class, "intake");
        shooter  = hardwareMap.get(DcMotor.class, "shooter");
        transfer = hardwareMap.get(DcMotor.class, "transfer");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addLine("18 Ball Auto Ready");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // ================= PRELOAD =================
        shootBalls(3);

        // ================= CYCLES =================
        // 5 cycles × 3 balls = 15 balls
        for (int cycle = 1; cycle <= 5 && opModeIsActive(); cycle++) {
            driveForward(0.5, 800);   // drive to stack
            intakeBalls(1200);        // pick up balls
            driveBackward(0.5, 800);  // drive back to goal
            shootBalls(3);            // shoot cycle
        }

        stopAll();
    }

    // ================= METHODS =================

    private void driveForward(double power, long timeMs) {
        setDrivePower(power, power);
        sleep(timeMs);
        stopDrive();
    }

    private void driveBackward(double power, long timeMs) {
        setDrivePower(-power, -power);
        sleep(timeMs);
        stopDrive();
    }

    private void setDrivePower(double left, double right) {
        leftFront.setPower(left);
        leftRear.setPower(left);
        rightFront.setPower(right);
        rightRear.setPower(right);
    }

    private void stopDrive() {
        setDrivePower(0, 0);
    }

    private void intakeBalls(long timeMs) {
        intake.setPower(1.0);
        sleep(timeMs);
        intake.setPower(0);
    }

    private void shootBalls(int count) {
        shooter.setPower(1.0); // spin up shooter
        sleep(800);            // wait for RPM

        for (int i = 0; i < count && opModeIsActive(); i++) {
            transfer.setPower(1.0);
            sleep(300);        // feed one ball
            transfer.setPower(0);
            sleep(250);        // spacing between shots
        }

        shooter.setPower(0);
    }

    private void stopAll() {
        stopDrive();
        intake.setPower(0);
        shooter.setPower(0);
        transfer.setPower(0);
    }
}
