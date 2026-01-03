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
public class AutonTester extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    private Robot robot;

    private boolean rapidShoot = false;

    private final ElapsedTime timer = new ElapsedTime();

    private DcMotorEx transfer;

    private int pathNumber = 67;
    String currentPath = "Path" + pathNumber;

    private boolean ran = true;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(125.6888888888889, 124.08888888888887, Math.toRadians(36)));

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
        panelsTelemetry.debug("Shooter rpm",robot.outtake.getRPM());
        panelsTelemetry.debug("Timer",timer.milliseconds());
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
                                    PedroPoses.START_POSE,
                                    PedroPoses.SHOOT_POSE
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(36))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    PedroPoses.SHOOT_POSE,
                                    PedroPoses.SECOND_INTAKE_CONTROL,
                                    PedroPoses.SECOND_INTAKE
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(0))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    PedroPoses.SECOND_INTAKE,
                                    PedroPoses.SHOOT_POSE
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(36))
                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    PedroPoses.SHOOT_POSE,
                                    PedroPoses.FIRST_INTAKE
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(0))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    PedroPoses.FIRST_INTAKE,
                                    PedroPoses.SHOOT_POSE
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(36))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    PedroPoses.SHOOT_POSE,
                                    PedroPoses.LAST_INTAKE_CONTROL,
                                    PedroPoses.LAST_INTAKE
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(0))

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    PedroPoses.LAST_INTAKE,
                                    PedroPoses.SHOOT_POSE
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(36))

                    .build();

        }
    }

    public void resetTimerAndPrepareShoot() {
        robot.outtake.spinToRpm(2400);
        robot.outtake.setLinkage(0.5);
        robot.intake.setPower(0.8);
        transfer.setPower(1.0);
        if (!follower.isBusy() && ran) {
            timer.reset();
            ran = false;
        }
    }

    public void shootAndAdvancePath(int pathNumber, PathChain currPath) {
        robot.outtake.spinToRpm(2500);
        if(!follower.isBusy()) {
            transfer.setPower(-1);
            robot.intake.setPower(1.0);
            telemetry.addData("Is in this loop",1);
            if(robot.outtake.getRPM()> 2400 || rapidShoot ) {
                robot.intake.setPower(1.0);
                transfer.setPower(-1.0);
                rapidShoot = true;
                if(timer.milliseconds() > 1200) {
                    pathState = pathNumber;
                    //follower.followPath(currPath);
                    ran = true;
                    rapidShoot = false;
                    return;
                }
            }

        }
    }

    public void intakeAndAdvance(int pathNumber, PathChain currPath) {
        robot.outtake.setLinkage(0.5);
        robot.outtake.setPower(-0.5);
        robot.intake.setPower(0.8);
        robot.outtake.spinToRpm(0);
        transfer.setPower(-0.05);
        if (transfer.getCurrent(CurrentUnit.AMPS) >= 5.0) {
            transfer.setPower(0);
        }

        if (!follower.isBusy()) {
            //follower.followPath(currPath);
            pathState = pathNumber;
            return;
        }

    }


    public int autonomousPathUpdate() {
        // Event markers will automatically trigger at their positions
        // Make sure to register NamedCommands in your RobotContainer
        switch(pathState) {
            case 0:
                // Start Path 1
                //follower.followPath(paths.Path1);

                resetTimerAndPrepareShoot();
                telemetry.addLine("Reset and Prepare Shoot Complete");
                shootAndAdvancePath(1, paths.Path1);
                telemetry.addLine("Complete Shoot and Advance");
                intakeAndAdvance(1, paths.Path1);
                telemetry.addLine("Complete Intake and Advance");
                follower.update();
                pathState = 67;
                ran = true;
                rapidShoot = false;
                break;

            case 1:
                telemetry.addData("Is Follower Busy? ", follower.isBusy());
                shootAndAdvancePath(2, paths.Path2);
                break;

            case 2:
                intakeAndAdvance(3, paths.Path3);
                resetTimerAndPrepareShoot();
                break;

            case 3:
                shootAndAdvancePath(4, paths.Path4);
                break;

            case 4:
                intakeAndAdvance(5, paths.Path5);

                resetTimerAndPrepareShoot();
                break;

            case 5:
                shootAndAdvancePath(6, paths.Path6);
                break;

            case 6:
                intakeAndAdvance(7, paths.Path7);

                resetTimerAndPrepareShoot();
                break;

            case 7:
                shootAndAdvancePath(8, paths.Path8);
                break;

            case 8:
                intakeAndAdvance(9, paths.Path9);

                resetTimerAndPrepareShoot();
                break;

            case 9:
                if(!follower.isBusy()) {
                    telemetry.addData("Is in this loop",1);
                    if(robot.outtake.getRPM()> 2400 || rapidShoot ) {
                        robot.intake.setPower(1.0);
                        transfer.setPower(-1.0);
                        rapidShoot = true;
                        if(timer.milliseconds() > 1000) {
                            pathState = 67;
                            ran = true;
                            rapidShoot = false;
                            break;
                        }
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
