package org.firstinspires.ftc.teamcode.opMode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.teamcode.PedroHelper.AutoBuilder;
import org.firstinspires.ftc.teamcode.PedroHelper.AutoRoutine;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;
import org.firstinspires.ftc.teamcode.subsystem.Turret;
import org.firstinspires.ftc.teamcode.util.PoseStorage;
import org.firstinspires.ftc.teamcode.util.TeamConstants;

@Autonomous(name = "PedroHelper", group = "Autonomous")
public class TestPedroHelper extends OpMode {

    private Follower follower;
    private TelemetryManager panelsTelemetry;

    private int pathState;
    private Timer pathTimer;
    private Intake intake;
    private Outtake outtake;


    private ElapsedTime actionTimer;
    private boolean ran = true;
    private boolean happened = false;
    private Turret turret;

    private DcMotorEx transfer;
    AutoRoutine autoBuilder;
    private final Pose startPose = new Pose(124, 119.527, 0.7009);
    private final Pose scorePose = new Pose(84, 84, Math.toRadians(0));
    private final Pose scorePoseEnd = new Pose(90, 110, 0);

    private final Pose Spike1End = new Pose(123.4, 83.472, 0);

    private final Pose Bez2Control = new Pose(85, 60, 0);
    private final Pose Spike2End = new Pose(133, 55, 0);

    private final Pose Bez3Control = new Pose(66.32625189681336, 28.748103186646432, 0);
    private final Pose Spike3End = new Pose(133, 35, 0);

    private final Pose Gate = new Pose(129, 61, Math.toRadians(31));
    private final Pose GateControl = new Pose(105.2443095599393, 59.28528072837631, 0);
    private final Pose GateBack = new Pose(132, 47.617602427921085, Math.toRadians(90));


    private Path scorePreload;

    // Paths
    private PathChain
            PrepSpike1, ScoreSpike1,
            PrepSpike2, ScoreSpike2,
            GoGate,BackGate,
            PrepSpike3, ScoreSpike3;
    private Servo blocker;

    // ---------------- Path Building ----------------
    private void buildPaths() {

        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        PrepSpike1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, Spike1End))
                .setLinearHeadingInterpolation(scorePose.getHeading(), Spike1End.getHeading(),0.6)
                .addParametricCallback(0.5 , this::prepareToShoot)
                .build();

        ScoreSpike1 = follower.pathBuilder()
                .addPath(new BezierLine(Spike1End, scorePose))
                .setLinearHeadingInterpolation(Spike1End.getHeading(), scorePose.getHeading())

                .build();

        GoGate = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, GateControl, Gate))
                .setLinearHeadingInterpolation(scorePose.getHeading(), Gate.getHeading())
//                .addPath(new BezierLine(Gate, GateBack))
//                .setLinearHeadingInterpolation(Gate.getHeading(), GateBack.getHeading())
                .build();

        BackGate = follower.pathBuilder()
                .addPath(new BezierLine(GateBack, scorePose))
                .setLinearHeadingInterpolation(Gate.getHeading(), scorePose.getHeading())
                .build();

        PrepSpike2 = follower.pathBuilder()


                .addPath(new BezierCurve(scorePose, Bez2Control, Spike2End))
                .setLinearHeadingInterpolation(scorePose.getHeading(), Spike2End.getHeading(),0.6)
                .build();

        ScoreSpike2 = follower.pathBuilder()
                .addPath(new BezierCurve(Spike2End, Bez2Control, scorePose))
                .setLinearHeadingInterpolation(Spike2End.getHeading(), scorePose.getHeading())

                .build();

        PrepSpike3 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, Bez3Control, Spike3End))
                .setLinearHeadingInterpolation(scorePose.getHeading(), Spike3End.getHeading(),0.6)
                .build();

        ScoreSpike3 = follower.pathBuilder()
                .addPath(new BezierLine(Spike3End, scorePoseEnd))
//                .setLinearHeadingInterpolation(Spike3End.getHeading(), scorePoseEnd.getHeading(),0.6)
                .setTangentHeadingInterpolation()
                .setReversed()

                .build();
    }

    // ---------------- Robot Actions ----------------
    private void prepareToShoot() {
        intake.setPower(TeamConstants.INTAKE_STOP);
        outtake.spinToRpm(3400);
//        blocker.setPosition(TeamConstants.BLOCKER_OPEN);
        transfer.setPower(TeamConstants.TRANSFER_CLOSED);
    }

    private void spinUpIntake() {
        outtake.spinToRpm(TeamConstants.outtake_Stop);
        intake.setPower(TeamConstants.INTAKE_INTAKE_POWER);
        transfer.setPower(TeamConstants.TRANSFER_INTAKE_POWER);
//        blocker.setPosition(TeamConstants.BLOCKER_CLOSE);
    }

    private void spinUpShooter() {
        telemetry.addLine("Ready to shoot");
        outtake.spinToRpm(3400);
    }

    private void spinUp(boolean withTransfer) {
        spinUpShooter();
        if (withTransfer) {
            transfer.setPower(TeamConstants.TRANSFER_IN_POWER);
            intake.setPower(TeamConstants.INTAKE_INTAKE_POWER);
            telemetry.addLine("transfer at 1");

        }else{
            intake.setPower(0);
            transfer.setPower(0);
            telemetry.addLine("transfer at 0");
        }
    }

    private void resetTimers() {
        if (!follower.isBusy() && ran) {
            actionTimer.reset();
            ran = false;
        }
    }

    private void resetBooleans() {
        ran = true;
        happened = false;
    }

    private void shoot() {
        if (follower.isBusy()) {
            prepareToShoot();
        }
        if (!follower.isBusy()) {

            if ((outtake.atSpeed(3350,3450)||happened) ) {
                happened = true;
                spinUp(true);
                if (actionTimer.milliseconds()>1200) {
                    pathState++;
                    resetBooleans();
                }
            } else {
                telemetry.addLine("Outtake not above"); // Code never reaches here
                spinUp(false);
            }
        }

    }



    private void spinIntakeGate(PathChain path) {
        spinUpIntake();
        if (!follower.isBusy()&&actionTimer.milliseconds()>1500) {
            follower.followPath(path,true);
            pathState++;
            resetBooleans();
        }
    }

    // ---------------- State Machine ----------------
    private void autonomousPathUpdate() {

    }

    // ---------------- OpMode Methods ----------------
    @Override
    public void init() {

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);
        turret = new Turret(hardwareMap,telemetry,follower);
        outtake = new Outtake(hardwareMap,telemetry);
        intake = new Intake(hardwareMap,telemetry,outtake,follower);
        transfer = hardwareMap.get(DcMotorEx.class, "Transfer");
        pathTimer = new Timer();
        actionTimer = new ElapsedTime();
//        blocker.setPosition(TeamConstants.BLOCKER_CLOSE);
        //transfer.setPower(TeamConstants.TRANSFER_IN_POWER);
        buildPaths();
        outtake.linkage.setPosition(TeamConstants.LINKAGE_SHOOT);
        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
        autoBuilder = new AutoBuilder(follower)
                .startAt(startPose)

                .drive(scorePreload)
                .onStart(() -> prepareToShoot())
                .onComplete(() -> spinUp(true))
                .waitMs(1500)

                .drive(PrepSpike2)
                .onStart(() -> stop())
                .atPercent(0.6, () -> spinUpIntake())
                .drive(ScoreSpike2)
                .onStart(() -> stop())
                .atPercent(0.6, () -> prepareToShoot())
                .onComplete(() -> spinUp(true))
                .waitMs(1500)

                .drive(PrepSpike1)
                .onStart(() -> stop())
                .atPercent(0.6, () -> spinUpIntake())
                .drive(ScoreSpike1)
                .onStart(() -> stop())
                .atPercent(0.6, () -> prepareToShoot())
                .onComplete(() -> spinUp(true))
                .waitMs(1500)

                .drive(PrepSpike3)
                .onStart(() -> stop())
                .atPercent(0.6, () -> spinUpIntake())
                .drive(ScoreSpike3)
                .onStart(() -> stop())
                .atPercent(0.6, () -> prepareToShoot())
                .onComplete(() -> spinUp(true))
                .waitMs(1500)

                .build();


//        blocker.setPosition(TeamConstants.BLOCKER_CLOSE);
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        pathState = 0;
    }

    @Override
    public void loop() {
        follower.update();
        autoBuilder.update();
        PoseStorage.pose = follower.getPose();

        // Telemetry
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.debug("Follower Busy", follower.isBusy());
        panelsTelemetry.debug("Shooter rpm",outtake.getRPM());
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void stop() {
        intake.setPower(0);
        transfer.setPower(0);
        outtake.setPower(0);
    }
}
