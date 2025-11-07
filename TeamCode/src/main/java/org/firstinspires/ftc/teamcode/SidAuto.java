package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Sid Auto  Blue", group = "")
@Configurable // Panels
public class SidAuto extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState = 1; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    DcMotor intake;
    DcMotorEx outtake;
    Servo linkage;

    PIDController rpmPID;

    ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(20, 124   , Math.toRadians(144)));

        paths = new Paths(follower); // Build paths

        intake = hardwareMap.get(DcMotor.class, "Intake");
        outtake = hardwareMap.get(DcMotorEx.class, "Outtake");
        linkage = hardwareMap.get(Servo.class, "Linkage");

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtake.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        outtake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        linkage.setPosition(0.8);

        rpmPID = new PIDController(ShooterConstants.kp, ShooterConstants.ki, ShooterConstants.kd);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {

        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;

        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(20.000, 124.000), new Pose(36.000, 108.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(135))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(36.000, 108.000),
                                    new Pose(72.000, 78.000),
                                    new Pose(24.000, 84.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(24.000, 84.000), new Pose(36.000, 108.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(36.000, 108.000),
                                    new Pose(80.000, 56.000),
                                    new Pose(24.000, 60.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(24.000, 60.000), new Pose(36.000, 108.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                    .build();

            Path6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(36.000, 108.000),
                                    new Pose(80.000, 28.000),
                                    new Pose(24.000, 36.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
                    .build();

            Path7 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(24.000, 36.000), new Pose(36.000, 108.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                    .build();

            Path8 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(36.000, 108.000), new Pose(48.000, 120.000))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(135))
                    .build();
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        timer.reset();
    }
    public int autonomousPathUpdate() {
        switch (pathState) {
            case 1:
                follower.followPath(paths.Path1);
                outtake(700);

                if (!follower.isBusy()) {
                    setPathState(2);
                }
                break;
            case 2:
                follower.followPath(paths.Path2);
                intake();

                if (!follower.isBusy()) {
                    setPathState(3);
                }
                break;
            case 3:
                follower.followPath(paths.Path3);
                outtake(2000);

                if (!follower.isBusy()) {
                    setPathState(4);
                }
                break;
            case 4:
                follower.followPath(paths.Path4);
                intake();

                if (!follower.isBusy()) {
                    setPathState(5);
                }
                break;
            case 5:
                follower.followPath(paths.Path5);
                outtake(2000);

                if (!follower.isBusy()) {
                    setPathState(6);
                }
                break;
            case 6:
                follower.followPath(paths.Path6);
                intake();

                if (!follower.isBusy()) {
                    setPathState(7);
                }
                break;
            case 7:
                follower.followPath(paths.Path7);
                outtake(2000);

                if (!follower.isBusy()) {
                    setPathState(8);
                }
                break;
            case 8:
                follower.followPath(paths.Path8);
                rest();

                if (!follower.isBusy()) {
                    setPathState(9);
                }
                break;
            case 9:
                follower.breakFollowing();
                break;
        }
        return pathState;
    }

    // ===== Intake/Outtake Logic =====
    public void intake() {
        intake.setPower(0.8);
        outtake.setPower(-0.2);
        linkage.setPosition(0.8);
    }

    public void outtake(int delayMs) {
        if (timer.milliseconds() < delayMs) {
            outtake.setPower(-0.8);
            intake.setPower(-0.5);
            linkage.setPosition(0.8);
        } else if (timer.milliseconds() < delayMs + 2800) {
            outtake.setPower(1.0);
            intake.setPower(0.0);
            linkage.setPosition(0.8);
        } else if (timer.milliseconds() < delayMs + 3300) {
            outtake.setVelocity(2400);
            linkage.setPosition(0.28);
            intake.setPower(0.0);
        } else {
            outtake.setVelocity(2400);
            linkage.setPosition(0.28);
            intake.setPower(0.8);
        }
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
