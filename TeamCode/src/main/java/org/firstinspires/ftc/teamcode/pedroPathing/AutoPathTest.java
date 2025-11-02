package org.firstinspires.ftc.teamcode.pedroPathing;

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

        // 2️⃣ Build your paths
        Paths paths = new Paths(follower);

        telemetry.addLine("Ready to run paths!");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            int currentPath = 1;

            while (opModeIsActive() && currentPath <= 3) {
                switch (currentPath) {
                    case 1:
                        telemetry.addData("Running", "Path1");
                        telemetry.update();
                        follower.followPath(paths.Path1);
                        break;
                    case 2:
                        telemetry.addData("Running", "Path2");
                        telemetry.update();
                        follower.followPath(paths.Path2);
                        break;
                    case 3:
                        telemetry.addData("Running", "Path3");
                        telemetry.update();
                        follower.followPath(paths.Path3);
                        break;
                    default:
                        telemetry.addLine("All paths complete!");
                        telemetry.update();
                        return;
                }

                while (follower.isBusy() && opModeIsActive()) {
                    follower.update();
                    telemetry.addData("Active Path", currentPath);
                    telemetry.update();
                }

                currentPath++;
            }

            telemetry.addLine("All paths finished!");
            telemetry.update();
        }
    }
}
