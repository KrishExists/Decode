package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.roadRunner.MecanumDrive;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class Drivetrain implements Subsystem {

    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;

    public MecanumDrive drive;

    private Pose2d currentPose;
    private PoseVelocity2d currentVelocity;

    PIDController headingController;
    public static double headingKp = 1.5;
    public static double headingKi = 0.0;
    public static double headingKd = 0.0;

    public Drivetrain(HardwareMap h, Telemetry t, Pose2d startPose) {

        this.hardwareMap = h;
        this.telemetry = t;

        drive = new MecanumDrive(hardwareMap, startPose);

        currentPose = startPose;
        currentVelocity = new PoseVelocity2d(new Vector2d(0,0), 0);

        headingController = new PIDController(headingKp, headingKi, headingKd);
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

    public void combinedDrive(Gamepad gamepad1, AprilTagDetection tag) {
//        Vector2d goalPose = NewRedTry.currentPose.position;
        Vector2d goalPose = new Vector2d(0,0);
        if (gamepad1.left_bumper) goalPose = new Vector2d(-72, -72); // blue
        if (gamepad1.right_bumper) goalPose = new Vector2d(-74, 72); // red

        double lockedHeading = Math.atan2(goalPose.y - currentPose.position.y, goalPose.x - currentPose.position.x);

        double error = AngleUnit.normalizeRadians(lockedHeading - AngleUnit.normalizeRadians(currentPose.heading.log()));
        double target = error + currentPose.heading.log();
        double output = Range.clip(headingController.calculate(currentPose.heading.log(), target), -1, 1);
        if(tag.metadata!=null){
             output = Range.clip(headingController.calculate(0,tag.ftcPose.bearing), -1, 1);

        }

        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x * 1.1

                ),
                (gamepad1.left_bumper || gamepad1.right_bumper) ? output : -gamepad1.right_stick_x
        ));
    }
    public void combinedDrive(Gamepad gamepad1) {
//        Vector2d goalPose = NewRedTry.currentPose.position;
        Vector2d goalPose = new Vector2d(0,0);
        if (gamepad1.left_bumper) goalPose = new Vector2d(-72, -72); // blue
        if (gamepad1.right_bumper) goalPose = new Vector2d(-74, 72); // red

        double lockedHeading = Math.atan2(goalPose.y - currentPose.position.y, goalPose.x - currentPose.position.x);

        double error = AngleUnit.normalizeRadians(lockedHeading - AngleUnit.normalizeRadians(currentPose.heading.log()));
        double target = error + currentPose.heading.log();
        double output = Range.clip(headingController.calculate(currentPose.heading.log(), target), -1, 1);

        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x * 1.1

                ),
                (gamepad1.left_bumper || gamepad1.right_bumper) ? output : -gamepad1.right_stick_x
        ));
    }

    private double normalize(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    public void moveRobot(double drive, double strafe, double turn) {
        drive = 0;
        double fl = drive + strafe + turn;
        double fr = drive - strafe - turn;
        double bl = drive - strafe + turn;
        double br = drive + strafe - turn;

        double max = Math.max(
                Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br))
        );

        if (max > 1.0) {
            fl /= max;
            fr /= max;
            bl /= max;
            br /= max;
        }
        setMotorPowers(fl,fr,bl,br);

    }

    private double angleToTarget(Pose2d current, Pose2d target) {
        double dx = target.position.x - current.position.x;
        double dy = target.position.y - current.position.y;
        return Math.atan2(dy, dx);
    }

    public void alignToPose(Pose2d target) {
        Pose2d current = currentPose;

        // Phase 1: rotate to face the target XY
        double angleToPoint = angleToTarget(current, target);
        double turnToFace = normalize(angleToPoint - current.heading.toDouble());

        // Phase 2: rotate to match final heading
        double turnToFinal = normalize(target.heading.toDouble() - current.heading.toDouble());

        double kP = 1.0;   // recommended tuning

        // If not facing the target point yet → turn toward it
        if (Math.abs(turnToFace) > Math.toRadians(3)) {
            double turn = kP * turnToFace;
            turn = Math.max(-0.6, Math.min(0.6, turn));  // clamp

            setMotorPowers(
                    turn,  // left front
                    -turn, // right front
                    turn,  // left back
                    -turn  // right back
            );
            return;
        }

        // Otherwise, match target heading
        double finalTurn = kP * turnToFinal;
        finalTurn = Math.max(-0.6, Math.min(0.6, finalTurn));

        setMotorPowers(
                finalTurn,
                -finalTurn,
                finalTurn,
                -finalTurn
        );
    }



    // Set specific motor powers (custom feed)
    public void setMotorPowers(double fl, double fr, double bl, double br) {
        drive.leftFront.setPower(fl);
        drive.rightFront.setPower(fr);
        drive.leftBack.setPower(bl);
        drive.rightBack.setPower(br);
    }

    public void updatePoseEstimate() {

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
    public void update(Gamepad gamepad1, Gamepad gamepad2) {
        update();
    }

    public void update() {
        currentVelocity = drive.updatePoseEstimate();
        currentPose = drive.localizer.getPose();

        Robot.pose = currentPose;
    }
}
