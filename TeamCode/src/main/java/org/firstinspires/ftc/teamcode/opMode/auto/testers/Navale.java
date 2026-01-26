package org.firstinspires.ftc.teamcode.opMode.auto.testers;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Autonomous
@Configurable
public class Navale extends OpMode {
    private Follower follower;
    private static Timer pathtimer;
    private Timer opmodetimer;

    public static void setPathState(PathState newState2){
        pathState = newState2;
        pathtimer.resetTimer();
    }

    @Override
    public void init() {
        pathState = PathState.DRIVE_STARTPOS_SHOOT_POS;
        pathtimer = new Timer();
        opmodetimer = new Timer();
        follower = Constants.createFollower(hardwareMap);

        buildPaths();
        follower.setPose(startPose);
    }

    public void start(){
        opmodetimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop(){
        follower.update();
        statePathUpdate();


        telemetry.addData("State of Path", pathState.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Path Time", pathtimer.getElapsedTime());

        telemetry.update();

    }


    public enum PathState {
        //START TO END
        //DRIVE
        //SHOOT
        DRIVE_STARTPOS_SHOOT_POS,
        SHOOT_PRELOAD,
        //States are used as different steps.
        //INTAKE_ARTIFACTS

    }


    static PathState pathState;

    private final Pose startPose = new Pose(62.93975903614457, 6.915662650602412, Math.toRadians(90));
    private final Pose shootPose = new Pose(61.077618164967554, 15.787534754402234, Math.toRadians(115));

    //private final Pose intakePose = new Pose(10.543, 9.376, Math.toRadians(-173));

    private PathChain driveStartPosShootPos;

    public void buildPaths(){
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
    }

    public void statePathUpdate() {
        switch(pathState) {
            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(driveStartPosShootPos, true);
                setPathState(PathState.SHOOT_PRELOAD); //reset timer and make a new state
                break;

            case SHOOT_PRELOAD:
                //check if follower did it's path.

                if (!follower.isBusy()) {
                    // ADD FLYWHEEL LOGIC
                    telemetry.addLine("DONE Path 1");
                    telemetry.update();
                    //transition to next state
                }
                break;
            //case INTAKE_ARTIFACTS:

            //if(!follower.isBusy()){
            //INTAKE LOGIC
            //telemetry.addLine("DONE PATH 2");
            //telemetry.update();
            //transition
            default:
                telemetry.addLine("NO STATE COMMANDED");
                break;
        }


    }
}
