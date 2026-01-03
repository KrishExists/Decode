package org.firstinspires.ftc.teamcode.opMode.auto;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystem.Robot;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable // Panels
public class NewPedroAuto extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    private int normalSpinToRpm = 2400;


    private Robot robot;

    private boolean rapidShoot = false;

    private final ElapsedTime timer = new ElapsedTime();

    private DcMotorEx transfer;

    private int pathNumber = 0;
    String currentPath = "Path" + pathNumber;

    private boolean ran = true;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths
        robot = new Robot(hardwareMap, telemetry);
        transfer = hardwareMap.get(DcMotorEx.class, "Transfer");


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
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(123.378, 127.289),

                                    new Pose(81.067, 83.822)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(36))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(81.067, 83.822),

                                    new Pose(131.333, 83.622)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(0))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(131.333, 83.622),

                                    new Pose(81.422, 83.733)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(36))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(81.422, 83.733),
                                    new Pose(85.156, 56.022),
                                    new Pose(135.467, 59.200)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(0))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(135.467, 59.200),
                                    new Pose(104.889, 60.222),
                                    new Pose(81.600, 83.911)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(36))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(81.600, 83.911),
                                    new Pose(69.278, 30.511),
                                    new Pose(134.956, 35.244)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(0))

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(134.956, 35.244),

                                    new Pose(80.978, 83.600)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(36))

                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(80.978, 83.600),

                                    new Pose(91.222, 75.222)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(36))

                    .build();
        }
    }


    public int autonomousPathUpdate() {
        switch(pathState) {
            case 0:
                follower.followPath(paths.Path1);
                pathState = 1;
                robot.outtake.spinToRpm(2500);
                robot.outtake.setLinkage(0.5);
                robot.intake.setPower(0.8);
                transfer.setPower(1.0);
                break;

            case 1:
                if(!follower.isBusy() && ran) {
                    timer.reset();
                    timer.startTime();
                    ran =false;
                }

                if(!follower.isBusy()) {
                    if(robot.outtake.getRPM() > normalSpinToRpm) {
                        normalSpinToRpm = 2300;
                        robot.outtake.spinToRpm(2500);
                        robot.outtake.setLinkage(0.5);
                        robot.intake.setPower(0.8);
                        transfer.setPower(-1.0);
                        if(timer.milliseconds() > 2000) {
                            follower.followPath(paths.Path2);
                            pathState = 2;
                            break;
                        }
                    }

                }

                break;

            case 2:
                robot.outtake.spinToRpm(0);
                robot.intake.setPower(0.8);
                transfer.setPower(-0.05);
                if(!follower.isBusy()) {
                    follower.followPath(paths.Path3);
                    pathState = 3;
                    normalSpinToRpm = 2400;
                }
                break;

            case 3:
                robot.outtake.spinToRpm(2500);
                robot.intake.setPower(0.8);
                transfer.setPower(1.0);
                if(!follower.isBusy() && ran) {
                    timer.reset();
                    timer.startTime();
                    ran =false;
                }

                if(!follower.isBusy()) {
                    if(robot.outtake.getRPM() > normalSpinToRpm) {
                        robot.outtake.spinToRpm(2500);
                        robot.outtake.setLinkage(0.5);
                        robot.intake.setPower(0.8);
                        transfer.setPower(-1.0);
                        if(timer.milliseconds() > 2000) {
                            follower.followPath(paths.Path4);
                            pathState = 4;
                            break;
                        }
                    }

                }
                break;

            case 4:
                robot.outtake.spinToRpm(0);
                robot.intake.setPower(0.8);
                transfer.setPower(-0.05);
                if(!follower.isBusy()) {
                    follower.followPath(paths.Path5);
                    pathState = 5;
                    normalSpinToRpm = 2400;
                }
                break;

            case 5:
                robot.outtake.spinToRpm(2500);
                robot.intake.setPower(0.8);
                transfer.setPower(1.0);
                if(!follower.isBusy() && ran) {
                    timer.reset();
                    timer.startTime();
                    ran =false;
                }

                if(!follower.isBusy()) {
                    if(robot.outtake.getRPM() > normalSpinToRpm) {
                        robot.outtake.spinToRpm(2500);
                        robot.outtake.setLinkage(0.5);
                        robot.intake.setPower(0.8);
                        transfer.setPower(-1.0);
                        if(timer.milliseconds() > 2000) {
                            follower.followPath(paths.Path6 );
                            pathState = 6;
                            break;
                        }
                    }

                }
                break;

            case 6:
                robot.outtake.spinToRpm(0);
                robot.intake.setPower(0.8);
                transfer.setPower(-0.05);
                if(!follower.isBusy()) {
                    follower.followPath(paths.Path7);
                    pathState = 7;
                    normalSpinToRpm = 2400;
                }
                break;

            case 7:
                robot.outtake.spinToRpm(2500);
                robot.intake.setPower(0.8);
                transfer.setPower(1.0);
                if(!follower.isBusy() && ran) {
                    timer.reset();
                    timer.startTime();
                    ran =false;
                }

                if(!follower.isBusy()) {
                    if(robot.outtake.getRPM() > normalSpinToRpm) {
                        robot.outtake.spinToRpm(2500);
                        robot.outtake.setLinkage(0.5);
                        robot.intake.setPower(0.8);
                        transfer.setPower(-1.0);
                        if(timer.milliseconds() > 2000) {
                            follower.followPath(paths.Path8);
                            pathState = 8;
                            break;
                        }
                    }

                }
                break;

            case 8:
                break;


        }
        return pathState;
    }


}