package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadRunner.MecanumDrive;

public class Drivetrain implements Subsystem {
    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    public MecanumDrive drive;

    // Cached pose & velocity
    private Pose2d currentPose;
    private PoseVelocity2d currentVelocity;

    public Drivetrain(HardwareMap h, Telemetry t, Pose2d pose) {
        this.hardwareMap = h;
        this.telemetry = t;

        drive = new MecanumDrive(hardwareMap, pose);

        currentPose = pose;
        currentVelocity = new PoseVelocity2d(new Vector2d(0, 0), 0);
    }

    public Drivetrain(HardwareMap h, Telemetry t) {
        this(h, t, new Pose2d(0, 0, 0));
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
    public void init() {

    }

    @Override
    public void update() {
        currentVelocity = drive.updatePoseEstimate();
        currentPose = drive.localizer.getPose();
    }
}
