package org.firstinspires.ftc.teamcode.opMode.auto.testers;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.TeamConstants;


@Autonomous
@Configurable
public class NavaleBroganTest extends OpMode {
    private Follower follower;
    private static Timer pathtimer;
    private Timer opmodetimer;
    private Servo blocker;

    private Outtake outtake;
    private Intake intake;
    private DcMotorEx transfer;


    public static void setPathState(PathState newState2) {
        pathState = newState2;
        pathtimer.resetTimer();
    }

    @Override
    public void init() {
        pathState = PathState.DRIVE_STARTPOS_SHOOT_POS;
        pathtimer = new Timer();
        opmodetimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        outtake = new Outtake(hardwareMap, telemetry);
        blocker = hardwareMap.get(Servo.class, "blocker");
        intake = new Intake(hardwareMap, telemetry, outtake);
        transfer = hardwareMap.get(DcMotorEx.class, "Transfer");
        buildPaths();
        follower.setPose(startPose);
    }

    public void start() {
        opmodetimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop() {
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
        SHOOT_POS_DRIVE_INTAKEPOS,
        INTAKE_ARTIFACTS

    }


    static PathState pathState;

    private final Pose startPose = new Pose(62.93975903614457, 6.915662650602412, Math.toRadians(90));
    private final Pose shootPose = new Pose(61.077618164967554, 15.787534754402234, Math.toRadians(115));
    private final Pose intakePose1 = new Pose(11.807228915662648, 14.67469879518073, Math.toRadians(190));
    private final Pose intakePose2 = new Pose(10.542168674698802, 9.518072289156645, Math.toRadians(190));



    private PathChain
            driveStartPosShootPos;
    private PathChain driveShootPosIntakePos;

    public void buildPaths() {
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        driveShootPosIntakePos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, intakePose1))
                .setLinearHeadingInterpolation(shootPose.getHeading(), intakePose1.getHeading())
                .addPath(new BezierLine(intakePose1, intakePose2))
                .setLinearHeadingInterpolation(intakePose1.getHeading(), intakePose2.getHeading())
                .build();

    }

    public void statePathUpdate() {
        switch (pathState) {
            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(driveStartPosShootPos, true);
                setPathState(PathState.SHOOT_PRELOAD); //reset timer and make a new state
                break;

            case SHOOT_PRELOAD:
                //check if follower did it's path.

                if (!follower.isBusy()) {
                    follower.followPath(driveShootPosIntakePos, true);
                    setPathState(null);

                }
                break;


            default:
                outtake.stop();
                intake.setPower(0);
                transfer.setPower(0);
                telemetry.addLine("NO STATE COMMANDED");
                break;
        }

    }
}

