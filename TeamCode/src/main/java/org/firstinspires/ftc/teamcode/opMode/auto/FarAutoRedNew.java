package org.firstinspires.ftc.teamcode.opMode.auto;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

@Autonomous(name = "FarAutoPahts", group = "Testers")
@Configurable // Panels
public class FarAutoRedNew extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(85.002,10.489 , Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

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
        public PathChain Park;
        public PathChain ShootFirst;

        public PathChain SpikeMarkPickup;
        public PathChain ShootSpike1;
        public PathChain HumanPickup;
        public PathChain LeaveHumanPlayer;
        public PathChain Park1;
        private Pose startPose = new Pose(85.002, 10.489);
        private Pose SpikePickControl = new Pose(96.916, 36.491);
        private Pose EndSpikePickup = new Pose(130.731, 36.874);
        private Pose ShootPose = new Pose(85.193, 10.818);
        private Pose EnterHuman = new Pose(136.904, 9.346);






        public Paths(Follower follower) {
            Park = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    ShootPose,
                                    new Pose(85.002, 10.489)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            SpikeMarkPickup = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    ShootPose,
                                    SpikePickControl,
                                    EndSpikePickup
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                    .build();

            ShootSpike1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    EndSpikePickup,
                                    ShootPose
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            HumanPickup = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    ShootPose,
                                    EnterHuman
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            LeaveHumanPlayer = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    EnterHuman,
                                    ShootPose
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();
            ShootFirst = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    startPose,
                                    ShootPose
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90),0)
                    .build();

        }
    }

    public int autonomousPathUpdate() {
        switch (pathState){
            case 0:
            follower.followPath(paths.ShootFirst);
            case 1:
                if(!follower.isBusy()){
                    follower.followPath(paths.SpikeMarkPickup);
                }
            case 2:
                if(!follower.isBusy()){
                    follower.followPath(paths.ShootSpike1);
                }
            case 3:
                if(!follower.isBusy()){
                    follower.followPath(paths.HumanPickup);
                }
            case 4:
                if(!follower.isBusy()){
                    follower.followPath(paths.LeaveHumanPlayer);
                }
            case 5:
                if(!follower.isBusy()){
                    follower.followPath(paths.LeaveHumanPlayer);
                }
            case 6:
                if(!follower.isBusy()){
                    follower.followPath(paths.Park);
                }
        }
        return pathState;
    }
}
