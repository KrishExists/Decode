//package org.firstinspires.ftc.teamcode.opMode.auto.testers;
//
//import com.bylazar.configurables.annotations.Configurable;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.PathChain;
//import com.pedropathing.util.Timer;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//
//
//import org.firstinspires.ftc.teamcode.subsystem.Intake;
//import org.firstinspires.ftc.teamcode.subsystem.Outtake;
//
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//
//
//@Autonomous
//@Configurable
//public class Navale extends OpMode {
//    private Follower follower;
//    private static Timer pathtimer;
//    private Timer opmodetimer;
//    private Outtake outtake;
//    private Intake intake;
//
//    public static void setPathState(PathState newState2){
//        pathState = newState2;
//        pathtimer.resetTimer();
//    }
//
//    @Override
//    public void init() {
//        pathState = PathState.DRIVE_STARTPOS_SHOOT_POS;
//        pathtimer = new Timer();
//        opmodetimer = new Timer();
//        follower = Constants.createFollower(hardwareMap);
//
//        buildPaths();
//        follower.setPose(startPose);
//    }
//
//    public void start(){
//        opmodetimer.resetTimer();
//        setPathState(pathState);
//    }
//
//    @Override
//    public void loop(){
//        follower.update();
//        statePathUpdate();
//
//
//        telemetry.addData("State of Path", pathState.toString());
//        telemetry.addData("x", follower.getPose().getX());
//        telemetry.addData("y", follower.getPose().getY());
//        telemetry.addData("heading", follower.getPose().getHeading());
//        telemetry.addData("Path Time", pathtimer.getElapsedTime());
//
//        telemetry.update();
//
//    }
//
//
//    public enum PathState {
//        //START TO END
//        //DRIVE
//        //SHOOT
//        DRIVE_STARTPOS_SHOOT_POS,
//        SHOOT_PRELOAD,
//        //States are used as different steps.
//        //INTAKE_ARTIFACTS
//
//    }
//
//
//    static PathState pathState;
//
//    private final Pose startPose = new Pose(62.93975903614457, 6.915662650602412, Math.toRadians(90));
//    private final Pose shootPose = new Pose(61.077618164967554, 15.787534754402234, Math.toRadians(115));
//
//    //private final Pose intakePose = new Pose(10.543, 9.376, Math.toRadians(-173));
//
//    private PathChain driveStartPosShootPos;
//
//    public void buildPaths(){
//        driveStartPosShootPos = follower.pathBuilder()
//                .addPath(new BezierLine(startPose, shootPose))
//                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
//                .build();
//
//
//    }
//
//    public void statePathUpdate() {
//        switch(pathState) {
//            case DRIVE_STARTPOS_SHOOT_POS:
//                follower.followPath(driveStartPosShootPos, true);
//                setPathState(PathState.SHOOT_PRELOAD); //reset timer and make a new state
//                break;
//
//            case SHOOT_PRELOAD:
//                //check if follower did it's path.
//
//                if (!follower.isBusy()) {
//                    outtake.spinToRpm(4000);
//                    outtake.setLinkage(0.42);
//
//                    if (outtake.atSpeed(3900,4000)){
//                        intake.setState(Intake.IntakeState.SpeedMid);
//                        //This transfers the ball  to shooter once rpm is ready.
//                    }
//
//                    // ADD FLYWHEEL LOGIC
//                    telemetry.addLine("DONE Path 1");
//                    telemetry.update();
//                    //transition to next state
//                }
//
//                break;
//            //case INTAKE_ARTIFACTS:
//
//            //if(!follower.isBusy()){
//            //INTAKE LOGIC
//            //telemetry.addLine("DONE PATH 2");
//            //telemetry.update();
//            //transition
//            default:
//                telemetry.addLine("NO STATE COMMANDED");
//                break;
//        }
//
//
//    }
//}


package org.firstinspires.ftc.teamcode.opMode.auto.testers;
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

@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable // Panels
public class Navale extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private PathChain paths ; // Paths defined in the Paths class

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new PathChain(follower.getConstraints()); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
         // Update autonomous state machine

        switch(pathState) {
            //add case statements and flywheel logic
        }


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

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(88.269, 7.615),
                                    new Pose(88.965, 9.494),
                                    new Pose(89.078, 9.799),
                                    new Pose(89.191, 10.105),
                                    new Pose(89.304, 10.410),
                                    new Pose(89.426, 10.739),
                                    new Pose(89.539, 11.044),
                                    new Pose(89.652, 11.349),
                                    new Pose(89.765, 11.654),
                                    new Pose(89.878, 11.960),
                                    new Pose(90.000, 12.288)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(70))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(90.000, 12.288),

                                    new Pose(132.058, 9.038)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(132.058, 9.038),

                                    new Pose(134.385, 10.058)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-3), Math.toRadians(350))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(134.385, 10.058),

                                    new Pose(89.923, 12.192)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(350), Math.toRadians(70))
                    .setReversed()
                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(89.923, 12.192),

                                    new Pose(126.596, 9.404)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(126.596, 9.404),

                                    new Pose(134.923, 9.192)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(134.923, 9.192),
                                    new Pose(127.489, 9.794),
                                    new Pose(119.522, 9.294),
                                    new Pose(113.223, 13.454),
                                    new Pose(104.758, 9.440),
                                    new Pose(97.654, 12.215),
                                    new Pose(90.077, 12.269)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(350), Math.toRadians(70))
                    .setReversed()
                    .build();
        }
    }

    private final Pose scorePose = new Pose(90, 12.288, Math.toRadians(70));
    private final Pose pickup1Pose = new Pose(132.0576923076923 , 9.038461538461565, Math.toRadians(-4));
    private final Pose pickup2Pose = new Pose(134.3846153846154,10.057692307692324, Math.toRadians(350));
    public void autonomousPathUpdate() {
        // Add your state machine Here
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
        paths = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        follower.followPath(paths);
    }
}

