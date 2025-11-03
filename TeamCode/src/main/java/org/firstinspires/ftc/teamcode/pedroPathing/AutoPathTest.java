package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.pedroPathing.Paths;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;  // 👈 your constants file

@Autonomous(name = "Pedro Path Test (Switch Version)", group = "Autonomous")
public class AutoPathTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        // 1️⃣ Build the follower using your Constants file
        Follower follower = Constants.createFollower(hardwareMap);
        Pose startPose = new Pose(127.51619870410367,124.4060475161987);
        follower.setStartingPose(startPose);

        // 2️⃣ Build your paths
        Paths paths = new Paths(follower);

        telemetry.addLine("Ready to run paths!");
        telemetry.update();
        PathChain Path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(127.51619870410367,124.4060475161987)
                                ,new Pose(70.0,80.0))
                )
                .setLinearHeadingInterpolation(Math.toRadians(215),Math.toRadians(67))
                .build();
        waitForStart();

            if (opModeIsActive()){
                follower.followPath(Path1);
            }

            telemetry.addLine("All paths finished!");
            telemetry.update();
        }
    }
