//package org.firstinspires.ftc.teamcode.opMode.auto;
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierCurve;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.Path;
//import com.pedropathing.paths.PathChain;
//import com.pedropathing.util.Timer;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import com.bylazar.telemetry.PanelsTelemetry;
//import com.bylazar.telemetry.TelemetryManager;
//
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//import org.firstinspires.ftc.teamcode.subsystem.Intake;
//import org.firstinspires.ftc.teamcode.subsystem.Outtake;
//import org.firstinspires.ftc.teamcode.util.PoseStorage;
//import org.firstinspires.ftc.teamcode.util.TeamConstants;
//
//@Autonomous(name = "Close Red (Version 2)", group = "Autonomous")
//public class DiffCloseRed extends OpMode {
//
//    private Follower follower;
//    private TelemetryManager panelsTelemetry;
//
//    private int pathState;
//    private Timer pathTimer;
//    private Intake intake;
//    private Outtake outtake;
//
//
//    private ElapsedTime actionTimer;
//    private boolean ran = true;
//    private boolean happened = false;
//
//    private boolean timerFallingEdge = true;
//
//    private DcMotorEx transfer;
//
//    private final Pose startPose = new Pose(125, 125, Math.toRadians(35));
//    private final Pose scorePose = new Pose(84, 84, Math.toRadians(45));
//    private final Pose scorePoseEnd = new Pose(90, 115, Math.toRadians(20));
//
//    private final Pose Bez1End = new Pose(98, 84, 0);
//    private final Pose Bez1Control = new Pose(85, 84, 0);
//    private final Pose Spike1End = new Pose(120, 84, 0);
//
//    private final Pose Bez2End = new Pose(98, 60, 0);
//    private final Pose Bez2Control = new Pose(85, 60, 0);
//    private final Pose Spike2End = new Pose(120, 60, 0);
//
//    private final Pose Bez3End = new Pose(98, 36, 0);
//    private final Pose Bez3Control = new Pose(78, 36, 0);
//    private final Pose Spike3End = new Pose(120, 36, 0);
//    private final Pose Gate = new Pose(127, 62, Math.toRadians(20));
//    private final Pose GateControl = new Pose(98, 69, 0);
//    private final Pose backGate = new Pose(96, 67, 0);
//
//    private Path scorePreload;
//
//    // Paths
//    private PathChain
//            PrepSpike1, FinishSpike1, ScoreSpike1,
//            PrepSpike2, FinishSpike2, ScoreSpike2,
//            GoGate, BackGate,
//            PrepSpike3, FinishSpike3, ScoreSpike3;
//    private Servo blocker;
//
//    // ---------------- Path Building ----------------
//    public void buildPaths() {
//        scorePreload = new Path(new BezierLine(startPose, scorePose));
//        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
//
//        PrepSpike1 = follower.pathBuilder()
//                .addPath(new BezierCurve(scorePose, Bez1Control, Bez1End))
//                .setLinearHeadingInterpolation(scorePose.getHeading(), Bez1End.getHeading())
//                .addPath(new BezierLine(Bez1End, Spike1End))
//                .setTangentHeadingInterpolation()
//                .build();
//
//        ScoreSpike1 = follower.pathBuilder()
//                .addPath(new BezierLine(Spike1End, scorePose))
//                .setLinearHeadingInterpolation(Spike1End.getHeading(), scorePose.getHeading())
//                .build();
//        GoGate = follower.pathBuilder()
//                .addPath(new BezierCurve(scorePose, GateControl, Gate))
//                .setLinearHeadingInterpolation(scorePose.getHeading(), Gate.getHeading())
//                .build();
//        BackGate = follower.pathBuilder()
//                .addPath(new BezierCurve(Gate, backGate, scorePose))
//                .setLinearHeadingInterpolation(Gate.getHeading(), scorePose.getHeading())
//                .build();
//
//        PrepSpike2 = follower.pathBuilder()
//                .addPath(new BezierCurve(scorePose, Bez2Control, Bez2End))
//                .setLinearHeadingInterpolation(scorePose.getHeading(), Bez2End.getHeading())
//                .addPath(new BezierLine(Bez2End, Spike2End))
//                .setTangentHeadingInterpolation()
//                .build();
//
//        ScoreSpike2 = follower.pathBuilder()
//                .addPath(new BezierLine(Spike2End, scorePose))
//                .setLinearHeadingInterpolation(Spike2End.getHeading(), scorePose.getHeading())
//                .build();
//
//        PrepSpike3 = follower.pathBuilder()
//                .addPath(new BezierCurve(scorePose, Bez3Control, Bez3End))
//                .setLinearHeadingInterpolation(scorePose.getHeading(), Bez3End.getHeading())
//                .addPath(new BezierLine(Bez3End, Spike3End))
//                .setTangentHeadingInterpolation()
//                .build();
//
//        ScoreSpike3 = follower.pathBuilder()
//                .addPath(new BezierLine(Spike3End, scorePoseEnd))
//                .setLinearHeadingInterpolation(Spike3End.getHeading(), scorePoseEnd.getHeading())
//                .build();
//    }
//
//
//    // ---------------- State Machine ----------------
//    private void autonomousPathUpdate() {
//        switch (pathState) {
//            case 0:
//                telemetry.addLine("Case 0 Started");
//                follower.followPath(scorePreload);
//                blocker.setPosition(TeamConstants.BLOCKER_OPEN);
//                transfer.setPower(TeamConstants.TRANSFER_REV);
//                intake.setPower(TeamConstants.INTAKE_FEED_POWER);
//                outtake.spinToRpm(TeamConstants.SHOOTER_MID_RPM);
//
//                if (!follower.isBusy() && timerFallingEdge) {
//                    actionTimer.reset();
//                    actionTimer.startTime();
//                    timerFallingEdge = false;
//                    transfer.setPower(TeamConstants.TRANSFER_IN_POWER);
//
//                }
//
//                if (actionTimer.milliseconds() >= 1000) {
//                    telemetry.addLine("Case 0 Ended");
//                    pathState++;
//                }
//                telemetry.addLine("Case 0 Ended");
//                break;
//
//            case 1:
//                telemetry.addLine("Case 1 Started");
//                follower.followPath(PrepSpike2);
//                blocker.setPosition(TeamConstants.BLOCKER_CLOSE);
//                intake.setPower(TeamConstants.INTAKE_IN_POWER);
//                transfer.setPower(TeamConstants.TRANSFER_IN_POWER);
//
//                if (!follower.isBusy()) {
//                    telemetry.addLine("Case 1 Ended");
//                    timerFallingEdge = true;
//                    pathState++;
//                }
//                break;
//
//            case 2:
//                telemetry.addLine("Case 2 Started");
//                follower.followPath(ScoreSpike2);
//                blocker.setPosition(TeamConstants.BLOCKER_OPEN);
//                transfer.setPower(TeamConstants.TRANSFER_REV);
//                intake.setPower(TeamConstants.INTAKE_FEED_POWER);
//                outtake.spinToRpm(TeamConstants.SHOOTER_MID_RPM);
//
//                if (!follower.isBusy() && timerFallingEdge) {
//                    actionTimer.reset();
//                    actionTimer.startTime();
//                    timerFallingEdge = false;
//                    transfer.setPower(TeamConstants.TRANSFER_IN_POWER);
//
//                }
//
//                if (actionTimer.milliseconds() >= 1000) {
//                    telemetry.addLine("Case 2 Ended");
//                    pathState++;
//                }
//                telemetry.addLine("Case 2 Ended");
//                break;
//
//            case 3:
//                telemetry.addLine("Case 3 Started");
//                follower.followPath(PrepSpike2);
//                blocker.setPosition(TeamConstants.BLOCKER_CLOSE);
//                intake.setPower(TeamConstants.INTAKE_IN_POWER);
//                transfer.setPower(TeamConstants.TRANSFER_IN_POWER);
//
//                if (!follower.isBusy()) {
//                    telemetry.addLine("Case 3 Ended");
//                    timerFallingEdge = true;
//                    pathState++;
//                }
//                break;
//            case 4:
//                telemetry.addLine("Case 4 Started");
//                follower.followPath(scorePreload);
//                telemetry.addLine("Case 4 Ended");
//                pathState++;
//                break;
//            case 5:
//                telemetry.addLine("Case 5 Started");
//                follower.followPath(scorePreload);
//                telemetry.addLine("Case 5 Ended");
//                pathState++;
//                break;
//            case 6:
//                telemetry.addLine("Case 6 Started");
//                follower.followPath(scorePreload);
//                telemetry.addLine("Case 6 Ended");
//                pathState++;
//                break;
//            case 7:
//                telemetry.addLine("Case 7 Started");
//                follower.followPath(scorePreload);
//                telemetry.addLine("Case 7 Ended");
//                pathState++;
//                break;
//            case 8:
//                telemetry.addLine("Case 8 Started");
//                follower.followPath(scorePreload);
//                telemetry.addLine("Case 8 Ended");
//                pathState++;
//                break;
//            case 9:
//                telemetry.addLine("Case 9 Started");
//                follower.followPath(scorePreload);
//                telemetry.addLine("Case 9 Ended");
//                pathState++;
//                break;
//            case 10:
//                telemetry.addLine("Case 10 Started");
//                follower.followPath(scorePreload);
//                telemetry.addLine("Case 10 Ended");
//                pathState++;
//                break;
//
//            default:
//                outtake.stop();
//                intake.setPower(TeamConstants.SHOOTER_CLOSED);
//        }
//    }
//
//    // ---------------- OpMode Methods ----------------
//    @Override
//    public void init() {
//        outtake = new Outtake(hardwareMap, telemetry);
//        intake = new Intake(hardwareMap, telemetry, outtake);
//        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
//        outtake.linkage.setPosition(TeamConstants.LINKAGE_SHOOT);
//
//        follower = Constants.createFollower(hardwareMap);
//        follower.setStartingPose(startPose);
//        follower.setPose(startPose);
//
//        transfer = hardwareMap.get(DcMotorEx.class, "Transfer");
//        blocker = hardwareMap.get(Servo.class, "blocker");
//
//        pathTimer = new Timer();
//        actionTimer = new ElapsedTime();
//        blocker.setPosition(0.5);
//
//        buildPaths();
//        pathState = 0;
//
//        panelsTelemetry.debug("Status", "Initialized");
//        panelsTelemetry.update(telemetry);
//    }
//
//    @Override
//    public void init_loop() {
//    }
//
//    @Override
//    public void start() {
//        pathState = 0;
//    }
//
//    @Override
//    public void loop() {
//        follower.update();
//        autonomousPathUpdate();
//        PoseStorage.pose = follower.getPose();
//
//        // Telemetry
//        panelsTelemetry.debug("Path State", pathState);
//        panelsTelemetry.debug("X", follower.getPose().getX());
//        panelsTelemetry.debug("Y", follower.getPose().getY());
//        panelsTelemetry.debug("Heading", Math.toDegrees(follower.getPose().getHeading()));
//        panelsTelemetry.debug("Follower Busy", follower.isBusy());
//        panelsTelemetry.debug("Shooter rpm", outtake.getRPM());
//        panelsTelemetry.debug("Transfer", transfer.getVelocity());
//        panelsTelemetry.update(telemetry);
//    }
//
//    @Override
//    public void stop() {
//    }
//}
