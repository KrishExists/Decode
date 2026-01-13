package org.firstinspires.ftc.teamcode.opMode.auto;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.teamcode.opMode.auto.pathsAuto.SimplySyncPaths;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.TeamConstants;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;


@Autonomous(name = "Simply Sync (Red)", group = "Autonomous")
@Configurable // Panels
public class RedSimplySynchronization extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private SimplySyncPaths paths; // Paths defined in the Paths class

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(80.178, 7.644,Math.toRadians(90)));

                paths = new SimplySyncPaths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        pathState = 0;
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





    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(paths.Path1);
                pathState++;
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path2);
                    pathState++;
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path3);
                    pathState++;
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path2);
                    pathState++;
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path3);
                    pathState++;
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path2);
                    pathState++;
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path3);
                    pathState++;
                }
                break;
            case 7:
                // Autonomous finished
                panelsTelemetry.debug("Status", "Finished");
                pathState = -1; // signal finished
                break;
        }

        return pathState;
    }



}
