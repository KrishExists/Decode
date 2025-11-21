package org.firstinspires.ftc.teamcode.opMode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.Robot;

@Autonomous
public class newREdTry extends LinearOpMode {
    private final ElapsedTime timer = new ElapsedTime();

    // ---- UPDATED TO MATCH YOUR MEEPMEEP FILE EXACTLY ----
    final Pose2d START_POSE = new Pose2d(-51.5, 51.5, Math.toRadians(-144));
    final Pose2d LSHOOT = new Pose2d(-22, 3, Math.toRadians(-235));

    final Pose2d GOSHOOT2 = new Pose2d(-23, 26, Math.toRadians(-235));
    final Pose2d SPIKE1 = new Pose2d(36, 51, Math.toRadians(-270));
    final Pose2d SPIKE2 = new Pose2d(14, 54, Math.toRadians(-270));
    final Pose2d SPIKE3 = new Pose2d(-10, 54, Math.toRadians(-270));

    Action shootPre, toSpike3, toShootFrom3, toSpike2, toShootFrom2, toSpike1, toShootFrom1, leave;
    boolean currentAction = true;

    public enum ShootStates {
        PRELOAD,
        CYCLE_3,
        SHOOT_3,
        CYCLE_2,
        SHOOT_2,
        CYCLE_1,
        SHOOT_1,
        LEAVE,
        END
    }

    ShootStates state = ShootStates.PRELOAD;

    Robot robot;

    // shot control flags used inside each SHOOT_* state to manage the staged, non-blocking shoot
    private boolean shotPreparing = false;
    private boolean shotFired = false;

    // Per-cycle configuration (tweak these values)
    private static final double PRELOAD_TARGET_RPM = 3000;   // preload spin-up RPM
    private static final double PRELOAD_READY_RPM = 2700;    // rpm threshold to start intake in preload
    private static final double CYCLE_TARGET_RPM = 1500;     // per-cycle shot RPM
    private static final double CYCLE_READY_RPM = 1300;      // threshold (slightly below target)
    private static final long LINKAGE_DOWN_MS = 250;         // how long we run intake with linkage down
    private static final long AFTER_FIRE_MS = 200;           // small delay after retracting linkage before ending state

    @Override
    public void runOpMode() throws InterruptedException {
        timer.reset();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = new Robot(hardwareMap, telemetry, START_POSE);

        build_paths();
        robot.init();

        waitForStart();
        timer.reset();

        while (opModeIsActive()) {
            if (isStopRequested()) return;

            robot.update();
            update();

            telemetry.addData("State", state);
            telemetry.addData("Outtake RPM (vel)", robot.outtake.getVelocity());
            telemetry.addData("Timer", timer.milliseconds());
            telemetry.update();
        }
    }

    // -------------------------------
    //     UPDATED PATH BUILDER
    // -------------------------------
    public void build_paths() {

        // PRELOAD → LSHOOT
        TrajectoryActionBuilder shootPrePath = robot.drive.drive.actionBuilder(START_POSE)
                .strafeToLinearHeading(LSHOOT.position, LSHOOT.heading);
        //LShoot -> GOFORWARD
        TrajectoryActionBuilder shootForward = robot.drive.drive.actionBuilder(LSHOOT)
                .strafeToLinearHeading(GOSHOOT2.position, GOSHOOT2.heading);

        // LSHOOT → SPIKE 3
        TrajectoryActionBuilder toSpike3Path = robot.drive.drive.actionBuilder(GOSHOOT2)
                .turnTo(Math.toRadians(-255))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(SPIKE3, Math.toRadians(90));

        // SPIKE 3 → LSHOOT
        TrajectoryActionBuilder toShootFrom3Path = robot.drive.drive.actionBuilder(SPIKE3)
                .strafeToLinearHeading(LSHOOT.position, LSHOOT.heading);

        // LSHOOT → SPIKE 2
        TrajectoryActionBuilder toSpike2Path = robot.drive.drive.actionBuilder(LSHOOT)
                .setTangent(Math.toRadians(20))
                .splineToSplineHeading(
                        new Pose2d(5, 20, Math.toRadians(-270)),
                        Math.toRadians(-20)
                )
                .splineToLinearHeading(SPIKE2, Math.toRadians(-270));

        // SPIKE 2 → LSHOOT
        TrajectoryActionBuilder toShootFrom2Path = robot.drive.drive.actionBuilder(SPIKE2)
                .strafeToLinearHeading(LSHOOT.position, LSHOOT.heading);

        // LSHOOT → SPIKE 1
        TrajectoryActionBuilder toSpike1Path = robot.drive.drive.actionBuilder(LSHOOT)
                .setTangent(Math.toRadians(15))
                .splineToSplineHeading(
                        new Pose2d(27, 10, Math.toRadians(-270)),
                        Math.toRadians(-15)
                )
                .splineToLinearHeading(SPIKE1, Math.toRadians(-270));

        // SPIKE 1 → LSHOOT
        TrajectoryActionBuilder toShootFrom1Path = robot.drive.drive.actionBuilder(SPIKE1)
                .strafeToLinearHeading(LSHOOT.position, LSHOOT.heading);

        // LEAVE
        TrajectoryActionBuilder leavePath = robot.drive.drive.actionBuilder(LSHOOT)
                .strafeTo(new Vector2d(0, 18));

        // BUILD ACTIONS
        shootPre = shootPrePath.build();
        toSpike3 = toSpike3Path.build();
        toShootFrom3 = toShootFrom3Path.build();
        toSpike2 = toSpike2Path.build();
        toShootFrom2 = toShootFrom2Path.build();
        toSpike1 = toSpike1Path.build();
        toShootFrom1 = toShootFrom1Path.build();
        leave = leavePath.build();
    }

    // ---------------------------------------------------
    //                 STATE MACHINE
    // ---------------------------------------------------
    public void update() {
        TelemetryPacket packet = new TelemetryPacket();

        switch (state) {
            case PRELOAD:
                // Run the preload path while spinning shooter to PRELOAD_TARGET_RPM
                currentAction = shootPre.run(packet);
                robot.outtake.spinToRpm(PRELOAD_TARGET_RPM);

                // once the path is complete, manage linkage/intake using RPM readiness
                if (!currentAction) {
                    // when shooter is almost up to speed, move linkage and feed preload
                    if (robot.outtake.upToRpm(PRELOAD_READY_RPM)) {
                        // lower linkage and run intake to seat preload
                        robot.outtake.setLinkage(0.6);
                        robot.intake.setPower(1.0);
                    } else {
                        // keep intake off until shooter is ready
                        robot.intake.setPower(0.0);
                    }
                }

                // transition after a bit of time on the preload (gives time to settle)
                if (!currentAction && timer.milliseconds() > 8000) {
                    state = newREdTry.ShootStates.CYCLE_3;
                    timer.reset();
                    timer.startTime();

                    // prepare for cycle: retract linkage and stop intake
                    robot.outtake.setLinkage(0.92);
                    robot.intake.setPower(0.0);

                    // clear shot flags
                    shotPreparing = false;
                    shotFired = false;
                }
                break;

            case CYCLE_3:
                // drive to spike 3 while getting ready
                robot.outtake.setLinkage(0.92);
                robot.outtake.setPower(-0.5);
                robot.intake.setPower(0.8);
                currentAction = toSpike3.run(packet);
                if (!currentAction) {
                    state = newREdTry.ShootStates.SHOOT_3;
                    timer.reset();
                    timer.startTime();

                    // reset staged-shot flags
                    shotPreparing = false;
                    shotFired = false;
                }
                break;

            case SHOOT_3:
                // We continuously run the return path while managing the shooter sequence non-blocking
                currentAction = toShootFrom3.run(packet);
                handleCycleShooting(CYCLE_TARGET_RPM, CYCLE_READY_RPM);

                // transition to next cycle when the return path has finished AND we've completed the shot
                if (!currentAction && shotFired && timer.milliseconds() > AFTER_FIRE_MS) {
                    state = newREdTry.ShootStates.CYCLE_2;
                    timer.reset();
                    timer.startTime();

                    // prepare for next cycle
                    robot.outtake.setLinkage(0.92);
                    robot.intake.setPower(0.0);
                    shotPreparing = false;
                    shotFired = false;
                }
                break;

            case CYCLE_2:
                robot.outtake.setLinkage(0.92);
                robot.outtake.setPower(-0.5);
                robot.intake.setPower(0.8);
                currentAction = toSpike2.run(packet);
                if (!currentAction) {
                    state = newREdTry.ShootStates.SHOOT_2;
                    timer.reset();
                    timer.startTime();
                    shotPreparing = false;
                    shotFired = false;
                }
                break;

            case SHOOT_2:
                currentAction = toShootFrom2.run(packet);
                handleCycleShooting(CYCLE_TARGET_RPM, CYCLE_READY_RPM);

                if (!currentAction && shotFired && timer.milliseconds() > AFTER_FIRE_MS) {
                    state = newREdTry.ShootStates.CYCLE_1;
                    timer.reset();
                    timer.startTime();

                    robot.outtake.setLinkage(0.92);
                    robot.intake.setPower(0.0);
                    shotPreparing = false;
                    shotFired = false;
                }
                break;

            case CYCLE_1:
                robot.outtake.setPower(-0.5);
                robot.intake.setPower(0.8);
                robot.outtake.setLinkage(0.92);
                currentAction = toSpike1.run(packet);
                if (!currentAction) {
                    state = newREdTry.ShootStates.SHOOT_1;
                    timer.reset();
                    timer.startTime();
                    shotPreparing = false;
                    shotFired = false;
                }
                break;

            case SHOOT_1:
                currentAction = toShootFrom1.run(packet);
                handleCycleShooting(CYCLE_TARGET_RPM, CYCLE_READY_RPM);

                if (!currentAction && shotFired && timer.milliseconds() > AFTER_FIRE_MS) {
                    state = newREdTry.ShootStates.LEAVE;
                    timer.reset();
                    timer.startTime();
                }
                break;

            case LEAVE:
                currentAction = leave.run(packet);
                if (!currentAction) {
                    state = newREdTry.ShootStates.END;
                }
                break;

            case END:
                // Optionally idle motors
                robot.intake.setPower(0.0);
                robot.outtake.setPower(0.0);
                break;
        }

        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    /**
     * handleCycleShooting:
     * Non-blocking staged shooting sequence used during SHOOT_3, SHOOT_2, SHOOT_1.
     *
     * Stages:
     *  1) Spin shooter to target RPM (caller should provide target and ready threshold)
     *  2) Once upToRpm(threshold) -> lower linkage (0.6) and start intake
     *  3) After LINKAGE_DOWN_MS -> stop intake, retract linkage to 0.92, mark shotFired and leave shooter spinning
     *  4) AFTER_FIRE_MS later the calling state may transition when return path finishes
     */
    private void handleCycleShooting(double targetRpm, double readyRpm) {
        // 1) Always attempt to spin to target
        robot.outtake.spinToRpm(targetRpm);

        // If we haven't started the "lower linkage and feed" stage yet
        if (!shotPreparing) {
            // Wait until shooter reports it's up to the ready threshold
            if (robot.outtake.upToRpm(readyRpm)) {
                // Lower linkage and start intake feed
                robot.outtake.setLinkage(0.6);
                robot.intake.setPower(1.0);
                shotPreparing = true;
                // restart timer to measure feeding duration
                timer.reset();
            } else {
                // ensure intake is not running prematurely
                robot.intake.setPower(0.0);
            }
        }
        // If linkage is down and feeding, wait LINKAGE_DOWN_MS then retract linkage and stop intake
        else if (shotPreparing && !shotFired) {
            if (timer.milliseconds() >= LINKAGE_DOWN_MS) {
                // stop feeding and retract
                robot.intake.setPower(0.0);
                robot.outtake.setLinkage(0.92);
                shotFired = true;
                // reset timer to give a small settling time before transition
                timer.reset();
            } else {
                // keep feeding while within the feed window
                robot.intake.setPower(1.0);
            }
        } else {
            // shot has been fired: keep shooter spinning at target (or adapt if you want to ramp down)
            robot.outtake.spinToRpm(targetRpm);
        }
    }
}
