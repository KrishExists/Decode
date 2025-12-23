package org.firstinspires.ftc.teamcode.opMode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.Robot;

@Autonomous(name = "MeepMeepConvertedAuto_Clean")
public class KrishRedDefWorks extends LinearOpMode {

    /* ================= POSES (MATCH MEEPMEEP) ================= */

    final Pose2d START_POSE = new Pose2d(-49.2, 50.1, Math.toRadians(-144));
    final Pose2d LSHOOT     = new Pose2d(-22, 3, Math.toRadians(-235));

    final Pose2d SPIKE3 = new Pose2d(-10, 52, Math.toRadians(-270));
    final Pose2d SPIKE2 = new Pose2d(16, 52, Math.toRadians(-270));
    final Pose2d SPIKE1 = new Pose2d(36, 51, Math.toRadians(-270));

    /* ================= HARDWARE ================= */

    MecanumDrive drive;
    Robot robot;
    DcMotorEx transfer;

    /* ================= ACTIONS ================= */

    Action toShootPre;
    Action toSpike3, backFrom3;
    Action toSpike2, backFrom2;
    Action toSpike1, backFrom1;
    Action leave;

    Action currentAction;

    /* ================= STATE ================= */

    enum AutoState {
        MOVE_TO_LSHOOT_PRELOAD,
        SHOOT_PRELOAD,

        MOVE_TO_SPIKE_3,
        INTAKE_SPIKE_3,
        MOVE_TO_LSHOOT_3,
        FEED_SHOOTER_3,

        MOVE_TO_SPIKE_2,
        INTAKE_SPIKE_2,
        MOVE_TO_LSHOOT_2,
        FEED_SHOOTER_2,

        MOVE_TO_SPIKE_1,
        INTAKE_SPIKE_1,
        MOVE_TO_LSHOOT_1,
        FEED_SHOOTER_1,

        LEAVE,
        END
    }

    AutoState state = AutoState.MOVE_TO_LSHOOT_PRELOAD;

    /* ================= FLAGS ================= */

    boolean actionStarted = false;
    boolean actionFinished = false;
    boolean ballDetected = false;
    boolean feedingStarted = false;

    /* ================= CONSTANTS ================= */

    static final double BALL_CURRENT_THRESHOLD = 5.0;
    static final int SHOOT_RPM = 3000;

    ElapsedTime timer = new ElapsedTime();

    /* ========================================================== */

    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

        drive = new MecanumDrive(hardwareMap, START_POSE);
        robot = new Robot(hardwareMap, telemetry, START_POSE);
        transfer = hardwareMap.get(DcMotorEx.class, "Transfer");

        buildPaths();

        waitForStart();
        timer.reset();

        while (opModeIsActive()) {

            TelemetryPacket packet = new TelemetryPacket();

            updateAction(packet);
            updateState();

            drive.updatePoseEstimate();
            Pose2d pose = drive.localizer.getPose();

            /* ================= TELEMETRY ================= */
            telemetry.addData("State", state);
            telemetry.addData("Pose", pose);
            telemetry.addData("Outtake RPM", robot.outtake.currentRPM());
            telemetry.addData("Transfer Current (A)",
                    transfer.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Timer (ms)", timer.milliseconds());
            telemetry.update();

            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }

    /* ================= PATHS ================= */

    void buildPaths() {

        toShootPre = drive.actionBuilder(START_POSE)
                .strafeToLinearHeading(LSHOOT.position, LSHOOT.heading)
                .build();

        toSpike3 = drive.actionBuilder(LSHOOT)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(SPIKE3, Math.toRadians(90))
                .build();

        backFrom3 = drive.actionBuilder(SPIKE3)
                .strafeToLinearHeading(LSHOOT.position, LSHOOT.heading)
                .build();

        toSpike2 = drive.actionBuilder(LSHOOT)
                .setTangent(Math.toRadians(20))
                .splineToLinearHeading(SPIKE2, Math.toRadians(-270))
                .build();

        backFrom2 = drive.actionBuilder(SPIKE2)
                .strafeToLinearHeading(LSHOOT.position, LSHOOT.heading)
                .build();

        toSpike1 = drive.actionBuilder(LSHOOT)
                .setTangent(Math.toRadians(15))
                .splineToLinearHeading(SPIKE1, Math.toRadians(-270))
                .build();

        backFrom1 = drive.actionBuilder(SPIKE1)
                .strafeToLinearHeading(LSHOOT.position, LSHOOT.heading)
                .build();

        leave = drive.actionBuilder(LSHOOT)
                .strafeTo(new Vector2d(0, 18))
                .build();
    }

    /* ================= ACTION HELPERS ================= */

    void startAction(Action action) {
        currentAction = action;
        actionStarted = true;
        actionFinished = false;
    }

    void updateAction(TelemetryPacket packet) {
        if (currentAction != null && !actionFinished) {
            boolean running = currentAction.run(packet);
            if (!running) actionFinished = true;
        }
    }

    /* ================= STATE MACHINE ================= */

    void updateState() {

        switch (state) {

            /* ---------- PRELOAD ---------- */

            case MOVE_TO_LSHOOT_PRELOAD:
                if (!actionStarted) {
                    startAction(toShootPre);

                    robot.outtake.spinToRpm(SHOOT_RPM);
                    robot.outtake.linkage.setPosition(0.5);

                    robot.intake.setPower(0);
                    transfer.setPower(0);
                }
                if (actionFinished) {
                    state = AutoState.SHOOT_PRELOAD;
                    timer.reset();
                }
                break;

            case SHOOT_PRELOAD:
                robot.intake.setPower(1);
                transfer.setPower(-1);


                if (timer.milliseconds() > 1000) {
                    robot.intake.setPower(0);
                    transfer.setPower(0);

                    state = AutoState.MOVE_TO_SPIKE_3;
                    actionStarted = false;
                }
                break;

            /* ---------- SPIKE 3 ---------- */

            case MOVE_TO_SPIKE_3:
                if (!actionStarted) {
                    startAction(toSpike3);
                    robot.outtake.stop();
                    state = AutoState.INTAKE_SPIKE_3;
                    ballDetected = false;

                }

                break;

            case INTAKE_SPIKE_3:
                robot.intake.setPower(1);
                if (!ballDetected) {
                    transfer.setPower(-1);
                    if (transfer.getCurrent(CurrentUnit.AMPS) > BALL_CURRENT_THRESHOLD) {
                        transfer.setPower(0);
                        ballDetected = true;
                    }
                }
                if (ballDetected) {
                    robot.intake.setPower(0);
                    state = AutoState.MOVE_TO_LSHOOT_3;
                    actionStarted = false;
                }
                break;

            case MOVE_TO_LSHOOT_3:
                if (!actionStarted) {
                    startAction(backFrom3);
                    robot.outtake.spinToRpm(SHOOT_RPM);
                    robot.outtake.linkage.setPosition(0.5);
                }
                if (actionFinished) {
                    state = AutoState.FEED_SHOOTER_3;
                    timer.reset();
                }
                break;

            case FEED_SHOOTER_3:
                robot.intake.setPower(1);
                transfer.setPower(-1);
                if (timer.milliseconds() > 600) {
                    robot.intake.setPower(0);
                    transfer.setPower(0);
                    state = AutoState.MOVE_TO_SPIKE_2;
                    actionStarted = false;
                }
                break;

            /* ---------- SPIKE 2 ---------- */

            case MOVE_TO_SPIKE_2:
                if (!actionStarted) {
                    startAction(toSpike2);
                    robot.outtake.stop();
                }
                if (actionFinished) {
                    state = AutoState.INTAKE_SPIKE_2;
                    ballDetected = false;
                }
                break;

            case INTAKE_SPIKE_2:
                robot.intake.setPower(1);
                if (!ballDetected) {
                    transfer.setPower(-1);
                    if (transfer.getCurrent(CurrentUnit.AMPS) > BALL_CURRENT_THRESHOLD) {
                        transfer.setPower(0);
                        ballDetected = true;
                    }
                }
                if (ballDetected) {
                    robot.intake.setPower(0);
                    state = AutoState.MOVE_TO_LSHOOT_2;
                    actionStarted = false;
                }
                break;

            case MOVE_TO_LSHOOT_2:
                if (!actionStarted) {
                    startAction(backFrom2);
                    robot.outtake.spinToRpm(SHOOT_RPM);
                    robot.outtake.linkage.setPosition(0.5);
                }
                if (actionFinished) {
                    state = AutoState.FEED_SHOOTER_2;
                    timer.reset();
                }
                break;

            case FEED_SHOOTER_2:
                robot.intake.setPower(1);
                transfer.setPower(-1);
                if (timer.milliseconds() > 600) {
                    robot.intake.setPower(0);
                    transfer.setPower(0);
                    state = AutoState.MOVE_TO_SPIKE_1;
                    actionStarted = false;
                }
                break;

            /* ---------- SPIKE 1 ---------- */

            case MOVE_TO_SPIKE_1:
                if (!actionStarted) {
                    startAction(toSpike1);
                    robot.outtake.stop();
                }
                if (actionFinished) {
                    state = AutoState.INTAKE_SPIKE_1;
                    ballDetected = false;
                }
                break;

            case INTAKE_SPIKE_1:
                robot.intake.setPower(1);
                if (!ballDetected) {
                    transfer.setPower(-1);
                    if (transfer.getCurrent(CurrentUnit.AMPS) > BALL_CURRENT_THRESHOLD) {
                        transfer.setPower(0);
                        ballDetected = true;
                    }
                }
                if (ballDetected) {
                    robot.intake.setPower(0);
                    state = AutoState.MOVE_TO_LSHOOT_1;
                    actionStarted = false;
                }
                break;

            case MOVE_TO_LSHOOT_1:
                if (!actionStarted) {
                    startAction(backFrom1);
                    robot.outtake.spinToRpm(SHOOT_RPM);
                    robot.outtake.linkage.setPosition(0.5);
                }
                if (actionFinished) {
                    state = AutoState.FEED_SHOOTER_1;
                    timer.reset();
                }
                break;

            case FEED_SHOOTER_1:
                robot.intake.setPower(1);
                transfer.setPower(-1);
                if (timer.milliseconds() > 600) {
                    robot.intake.setPower(0);
                    transfer.setPower(0);
                    state = AutoState.LEAVE;
                    actionStarted = false;
                }
                break;

            /* ---------- LEAVE ---------- */

            case LEAVE:
                if (!actionStarted) {
                    startAction(leave);
                }
                if (actionFinished) {
                    state = AutoState.END;
                }
                break;

            case END:
                robot.intake.setPower(0);
                transfer.setPower(0);
                robot.outtake.stop();
                break;
        }
    }
}
