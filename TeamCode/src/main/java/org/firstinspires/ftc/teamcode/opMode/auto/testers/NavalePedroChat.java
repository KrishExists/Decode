
package org.firstinspires.ftc.teamcode.opMode.auto.testers;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;
import org.firstinspires.ftc.teamcode.util.TeamConstants;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Pedro Pathing Autonomous navalechat", group = "Autonomous")
@Configurable // Panels
public class NavalePedroChat extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    private Outtake outtake;
    private Servo blocker;
    private DcMotorEx transfer;
    private Intake intake;
    private static final int START = 0;
    private static final int DRIVE_TO_SCORE = 1;
    private static final int OUTTAKE = 2;
    private static final int PARK = 3;


    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths
        pathState = START;

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
        public PathChain Path3;
        public PathChain Path1;
        public PathChain Path2;

        public Paths(Follower follower) {
            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(82.193, 7.373),

                                    new Pose(84.205, 13.241)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(65))

                    .build();

            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(84.205, 13.241),

                                    new Pose(131.904, 12.145)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(65), Math.toRadians(0))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(131.904, 12.145),

                                    new Pose(134.892, 9.265)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(350))

                    .build();
        }
    }

    public void prepareToShoot(){
        intake.setPower(TeamConstants.INTAKE_FEED_POWER);
        outtake.spinToRpm(TeamConstants.SHOOTER_MID_RPM);
        blocker.setPosition(TeamConstants.BLOCKER_CLOSE);
        transfer.setPower(-1);
        telemetry.addLine("transfer power 0");
    }


    public int autonomousPathUpdate() {

            switch (pathState) {

                case START:
                    follower.followPath(paths.Path3);
                    pathState = DRIVE_TO_SCORE;
                    break;

                case DRIVE_TO_SCORE:
                    if (!follower.isBusy()) {
                        follower.followPath(paths.Path1);
                        pathState = OUTTAKE;
                    }
                    break;

                case OUTTAKE:
                    //OUTTAKE LOGIC GOES HERE
                    outtake.spinToRpm(2850);
                    outtake.setLinkage(0.5);
                    if (!follower.isBusy()) {
                        prepareToShoot();
                        follower.followPath(paths.Path2);
                        pathState = PARK;
                    }
                    break;

                case PARK:
                    break;
            }
            return pathState;
        }

    }