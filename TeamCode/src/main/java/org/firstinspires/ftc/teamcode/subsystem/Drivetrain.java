package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
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
    public void manualDrive(Gamepad g) {
        double y = -g.left_stick_y;      // forward/back
        double x = g.left_stick_x * 1.1; // strafe with 1.1 correction
        double rx = g.right_stick_x;     // rotation

        double denom = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);

        double fl = (y + x + rx) / denom;
        double bl = (y - x + rx) / denom;
        double fr = (y - x - rx) / denom;
        double br = (y + x - rx) / denom;

        // Send directly to Road Runner's drive class
        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(fl, fr),  // Road Runner interprets this correctly
                rx                     // rotation
        ));
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
