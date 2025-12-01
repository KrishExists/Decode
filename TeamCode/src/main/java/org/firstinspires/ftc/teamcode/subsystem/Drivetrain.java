package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadRunner.MecanumDrive;

public class Drivetrain implements Subsystem {

    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;

    public MecanumDrive drive;

    private Pose2d currentPose;
    private PoseVelocity2d currentVelocity;

    public Drivetrain(HardwareMap h, Telemetry t, Pose2d startPose) {
        this.hardwareMap = h;
        this.telemetry = t;

        drive = new MecanumDrive(hardwareMap, startPose);

        currentPose = startPose;
        currentVelocity = new PoseVelocity2d(new Vector2d(0,0), 0);
    }

    public Drivetrain(HardwareMap h, Telemetry t) {
        this(h, t, new Pose2d(0, 0, 0));
    }

    // ----------------------------------------
    // MANUAL DRIVE (converted from your old Drive.java)
    // ----------------------------------------
    public void manualDrive(Gamepad gamepad1) {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1;
        double rx = gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);

        drive.leftFront.setPower((y + x + rx) / denominator);
        drive.leftBack.setPower((y - x + rx) / denominator);
        drive.rightFront.setPower((y - x - rx) / denominator);
        drive.rightBack.setPower((y + x - rx) / denominator);
    }

    // Set specific motor powers (custom feed)
    public void setMotorPowers(double fl, double fr, double bl, double br) {
        drive.leftFront.setPower(fl);
        drive.rightFront.setPower(fr);
        drive.leftBack.setPower(bl);
        drive.rightBack.setPower(br);
    }

    // Stop drivetrain
    public void stop() {
        setMotorPowers(0, 0, 0, 0);
    }

    public void setPose(Pose2d pose) {
        drive.localizer.setPose(pose);
    }

    public Pose2d getPose() {
        return currentPose;
    }

    public PoseVelocity2d getPoseVelocity() {
        return currentVelocity;
    }

    @Override
    public void init() {}


    @Override
    public void update(Gamepad gamepad2) {
        currentVelocity = drive.updatePoseEstimate();
        currentPose = drive.localizer.getPose();
    }
}
