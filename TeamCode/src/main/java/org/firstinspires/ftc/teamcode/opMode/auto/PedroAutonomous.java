package org.firstinspires.ftc.teamcode.opMode.auto;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
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
public class PedroAutonomous extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    private Robot robot;

    private final ElapsedTime timer = new ElapsedTime();

    private DcMotorEx transfer;

    private boolean ran = true;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths
        robot = new Robot(hardwareMap, telemetry);


        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
        transfer = hardwareMap.get(DcMotorEx.class, "Transfer");

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
        public PathChain Path9;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(123.733, 123.022),

                                    new Pose(79.644, 83.822)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(36))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(79.644, 83.822),
                                    new Pose(70.944, 54.411),
                                    new Pose(135.267, 57.400)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(0))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(135.267, 57.400),
                                    new Pose(102.300, 57.500),
                                    new Pose(79.467, 82.267)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(36))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(79.467, 82.267),

                                    new Pose(131.444, 62.044)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(15))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(131.444, 62.044),

                                    new Pose(80.022, 82.667)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(15), Math.toRadians(36))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(80.022, 82.667),
                                    new Pose(88.467, 83.800),
                                    new Pose(128.422, 84.133)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(0))

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(128.422, 84.133),

                                    new Pose(80.000, 85.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(36))

                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(80.000, 85.000),
                                    new Pose(63.789, 25.922),
                                    new Pose(133.889, 34.756)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(0))

                    .build();

            Path9 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(133.889, 34.756),

                                    new Pose(79.911, 84.822)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(36))

                    .build();
        }
    }


    public int autonomousPathUpdate() {
        // Event markers will automatically trigger at their positions
        // Make sure to register NamedCommands in your RobotContainer
        switch(pathState) {
            case 0:
                // Start Path 1
                follower.followPath(paths.Path1);
                pathState = 1;
                break;

            case 1:
                robot.outtake.spinToRpm(2500);
                robot.outtake.setLinkage(0.5);
                robot.intake.setPower(0.8);
                transfer.setPower(1.0);
                if(!follower.isBusy()) {
                    timer.reset();
                    timer.startTime();
                    if(robot.outtake.getRPM()> 2400) {
                        robot.intake.setPower(1.0);
                        transfer.setPower(-1.0);
                    }
                    if(timer.milliseconds() > 650) {
                        pathState = 2;
                        follower.followPath(paths.Path2);
                        break;
                    }
                }

                break;

            case 2:
                robot.outtake.setLinkage(0.5);
                robot.outtake.setPower(-0.5);
                robot.intake.setPower(0.8);
                robot.outtake.spinToRpm(0);
                transfer.setPower(0);
                if (transfer.getCurrent(CurrentUnit.AMPS) >= 5.0) {
                    transfer.setPower(0);
                }

                if (!follower.isBusy()) {
                    follower.followPath(paths.Path3);
                    pathState = 3;
                }
                break;

            case 3:
                robot.outtake.setLinkage(0.5);
                robot.outtake.spinToRpm(2500);
                robot.intake.setPower(0.8);
                transfer.setPower(1.0);

                if(!follower.isBusy()) {
                    timer.reset();
                    timer.startTime();
                    if(robot.outtake.getRPM()> 2400) {
                        robot.intake.setPower(1.0);
                        transfer.setPower(-1.0);
                    }
                    if(timer.milliseconds() > 650) {
                        follower.followPath(paths.Path4);
                        pathState = 4;
                        break;
                    }
                }

                break;

            case 4:
                robot.outtake.setLinkage(0.5);
                robot.outtake.setPower(-0.5);
                robot.intake.setPower(0.8);
                robot.outtake.spinToRpm(0);
                transfer.setPower(0);
                if (transfer.getCurrent(CurrentUnit.AMPS) >= 5.0) {
                    transfer.setPower(0);
                }

                if (!follower.isBusy() && ran) {
                    timer.reset();
                    timer.startTime();
                    ran = false;
                }

                if(!follower.isBusy() && timer.milliseconds()>1000) {
                    follower.followPath(paths.Path5);
                    pathState = 5;
                }
                break;

            case 5:
                robot.outtake.setLinkage(0.5);
                robot.outtake.spinToRpm(2500);
                robot.intake.setPower(0.8);
                transfer.setPower(1.0);

                if(!follower.isBusy()) {
                    timer.reset();
                    timer.startTime();
                    if(robot.outtake.getRPM()> 2400) {
                        robot.intake.setPower(1.0);
                        transfer.setPower(-1.0);
                    }
                    if(timer.milliseconds() > 650) {
                        follower.followPath(paths.Path6);
                        pathState = 6;
                        break;
                    }
                }

                break;

            case 6:
                robot.outtake.setLinkage(0.5);
                robot.outtake.setPower(-0.5);
                robot.intake.setPower(0.8);
                robot.outtake.spinToRpm(0);
                transfer.setPower(0);
                if (transfer.getCurrent(CurrentUnit.AMPS) >= 5.0) {
                    transfer.setPower(0);
                }

                if (!follower.isBusy()) {
                    follower.followPath(paths.Path7);
                    pathState = 7;
                }
                break;

            case 7:
                robot.outtake.setLinkage(0.5);
                robot.outtake.spinToRpm(2500);
                robot.intake.setPower(0.8);
                transfer.setPower(1.0);

                if(!follower.isBusy()) {
                    timer.reset();
                    timer.startTime();
                    if(robot.outtake.getRPM()> 2400) {
                        robot.intake.setPower(1.0);
                        transfer.setPower(-1.0);
                    }
                    if(timer.milliseconds() > 650) {
                        follower.followPath(paths.Path8);
                        pathState = 8;
                        break;
                    }
                }

                break;

            case 8:
                robot.outtake.setLinkage(0.5);
                robot.outtake.setPower(-0.5);
                robot.intake.setPower(0.8);
                robot.outtake.spinToRpm(0);
                transfer.setPower(0);
                if (transfer.getCurrent(CurrentUnit.AMPS) >= 5.0) {
                    transfer.setPower(0);
                }

                if (!follower.isBusy()) {
                    follower.followPath(paths.Path9);
                    pathState = 9;
                }
                break;

            case 9:
                robot.outtake.setLinkage(0.5);
                robot.outtake.spinToRpm(2500);
                robot.intake.setPower(0.8);
                transfer.setPower(1.0);

                if(!follower.isBusy()) {
                    timer.reset();
                    timer.startTime();
                    if(robot.outtake.getRPM()> 2400) {
                        robot.intake.setPower(1.0);
                        transfer.setPower(-1.0);
                    }
                    if(timer.milliseconds() > 650) {
                        pathState = 67;
                        break;
                    }
                }

                break;



        }
        return pathState;
    }


}



//Pedro Pathing Visualizer
//27.0s / 27.0s
//        (572 in)
//
//
//
//
//
//
//
//
//
//Current Robot Position
//X:
//        79.911
//Y:
//        84.822
//Heading:
//        36°
//Starting Point
//X:
//        123.73333333333333
//Y:
//        123.02222222222221
//Path 1
//Path 1
//
//
//
//
//
//
//
//
//Point Position:
//X:
//        79.64444444444443
//Y:
//        83.8222222222222
//
//Linear
//Start:
//        36
//End:
//        36
//Control Points (0)
//Path 2
//Path 2
//
//
//
//
//
//
//
//
//Point Position:
//X:
//        135.2666666666667
//Y:
//        57.40000000000001
//
//Linear
//Start:
//        36
//End:
//        0
//Control Points (1)
//Control Point 1
//
//X:
//        70.94444444444444
//Y:
//        54.4111111111111
//Line 2, Control Point 1
//Path 3
//Path 3
//
//
//
//
//
//
//
//
//Point Position:
//X:
//        79.46666666666667
//Y:
//        82.26666666666668
//
//Linear
//Start:
//        0
//End:
//        36
//Control Points (1)
//Control Point 1
//
//X:
//        102.30000000000001
//Y:
//        57.5
//Line 3, Control Point 1
//Path 4
//Path 4
//
//
//
//
//
//
//
//
//Point Position:
//X:
//        131.44444444444443
//Y:
//        62.04444444444445
//
//Linear
//Start:
//        36
//End:
//        15
//Control Points (0)
//Wait
//        Wait
//1000
//ms
//
//
//
//
//
//
//Path 5
//Path 5
//
//
//
//
//
//
//
//
//Point Position:
//X:
//        80.02222222222223
//Y:
//        82.66666666666667
//
//Linear
//Start:
//        15
//End:
//        36
//Control Points (0)
//Path 6
//Path 6
//
//
//
//
//
//
//
//
//Point Position:
//X:
//        128.42222222222222
//Y:
//        84.13333333333335
//
//Linear
//Start:
//        36
//End:
//        0
//Control Points (1)
//Control Point 1
//
//X:
//        88.46666666666667
//Y:
//        83.8
//Line 6, Control Point 1
//Path 7
//Path 7
//
//
//
//
//
//
//
//
//Point Position:
//X:
//        80
//Y:
//        85
//
//Linear
//Start:
//        0
//End:
//        36
//Control Points (0)
//Path 8
//Path 8
//
//
//
//
//
//
//
//
//Point Position:
//X:
//        133.88888888888889
//Y:
//        34.755555555555574
//
//Linear
//Start:
//        36
//End:
//        0
//Control Points (1)
//Control Point 1
//
//X:
//        63.78888888888886
//Y:
//        25.92222222222225
//Line 8, Control Point 1
//Path 9
//Path 9
//
//
//
//
//
//
//
//
//Point Position:
//X:
//        79.91111111111113
//Y:
//        84.82222222222222
//
//Linear
//Start:
//        0
//End:
//        36
//Control Points (0)
//
//Add Path
//
//
//Add Wait
//
//
//
//
