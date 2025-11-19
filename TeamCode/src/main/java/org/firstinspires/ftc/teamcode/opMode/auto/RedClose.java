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
public class RedClose extends LinearOpMode {
    private final ElapsedTime timer = new ElapsedTime();

    // ---- UPDATED TO MATCH YOUR MEEPMEEP FILE EXACTLY ----
    final Pose2d START_POSE = new Pose2d(-51.5, 51.5, Math.toRadians(-144));
    final Pose2d LSHOOT = new Pose2d(-22, 3, Math.toRadians(-235));
    final Pose2d SPIKE1 = new Pose2d(36, 51, Math.toRadians(-270));
    final Pose2d SPIKE2 = new Pose2d(14, 54, Math.toRadians(-270));
    final Pose2d SPIKE3 = new Pose2d(-10, 42, Math.toRadians(-270));

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
            telemetry.addData("Outtake", robot.outtake.getVelocity());
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

        // LSHOOT → SPIKE 3
        TrajectoryActionBuilder toSpike3Path = robot.drive.drive.actionBuilder(LSHOOT)
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
    //     (UNCHANGED — no edits to your timing logic)
    // ---------------------------------------------------
    public void update() {
        TelemetryPacket packet = new TelemetryPacket();

        switch (state) {
            case PRELOAD:

                if(timer.milliseconds() < 3000) {
                    robot.outtake.spinToRpm(1700);
                }

                else if(timer.milliseconds() < 3500) {
                    robot.outtake.setLinkage(0.6);
                    robot.intake.setPower(0);
                }


                else if(timer.milliseconds() < 4500){
                    robot.outtake.setLinkage(0.6);
                    robot.intake.setPower(0.9);
                }

                else if(timer.milliseconds()<5500){
                    robot.intake.setPower(0);
                    robot.outtake.spinToRpm(2000);
                }else{
                    robot.intake.setPower(0.9);
                }

                currentAction = shootPre.run(packet);

                if (!currentAction && timer.milliseconds() > 7000) {
                    state = RedClose.ShootStates.CYCLE_3;
                    timer.reset();
                    timer.startTime();
                    robot.outtake.setLinkage(0.92);
                    robot.intake.setPower(0);
                }
                break;
            case CYCLE_3:
                robot.outtake.setLinkage(0.92);
                robot.outtake.setPower(-0.5);
                robot.intake.setPower(0.8);
                currentAction = toSpike3.run(packet);
                if (!currentAction) {
                    state = RedClose.ShootStates.SHOOT_3;
                    timer.reset();
                    timer.startTime();
                }
                break;
            case SHOOT_3:
                if(timer.milliseconds()<600){
                    robot.outtake.setPower(-1);
                    robot.intake.setPower(-0.6);
                }
                if(timer.milliseconds() < 2000) {
                    robot.outtake.spinToRpm(3000);
                }

                else if(timer.milliseconds() < 2500) {
                    robot.outtake.setLinkage(0.6);
                    robot.intake.setPower(0);
                }


                else if(timer.milliseconds() < 3500){
                    robot.intake.setPower(1);
                }

                else if(timer.milliseconds()<4000){
                    robot.intake.setPower(0);
                    robot.outtake.spinToRpm(2000);
                }else{
                    robot.intake.setPower(1);
                }
                currentAction = toShootFrom3.run(packet);

                if (!currentAction&&timer.milliseconds()>6000) {
                    state = RedClose.ShootStates.CYCLE_2;
                    timer.reset();
                    timer.startTime();
                    robot.outtake.setLinkage(0.92);
                    robot.intake.setPower(0);

                }
                break;
            case CYCLE_2:
                robot.outtake.setLinkage(0.92);
                robot.outtake.setPower(-0.5);
                robot.intake.setPower(0.8);

                currentAction = toSpike2.run(packet);

                if (!currentAction) {
                    state = RedClose.ShootStates.SHOOT_2;
                    timer.reset();
                    timer.startTime();

                }
                break;
            case SHOOT_2:
                if(timer.milliseconds()<600){
                    robot.outtake.setPower(-0.6);
                    robot.intake.setPower(-0.2);
                }
//                else{
//                    robot.outtake.spinToRpm(2500);
//                    if(robot.outtake.upToRpm(2200)){
//                        robot.outtake.linkage.setPosition(0.6);
//                        robot.intake.setPower(0.8);
//
//                    }
//                }
                else if(timer.milliseconds() < 2000) {
                    robot.outtake.spinToRpm(3000);
                }

                else if(timer.milliseconds() < 2500) {
                    robot.outtake.setLinkage(0.6);
                    robot.intake.setPower(0);
                }


                else if(timer.milliseconds() < 3500){
                    robot.outtake.setLinkage(0.6);
                    robot.intake.setPower(1);
                }

                else if(timer.milliseconds()<4000){
                    robot.intake.setPower(0);
                    robot.outtake.spinToRpm(2000);
                }else{
                    robot.intake.setPower(1);
                }
                currentAction = toShootFrom2.run(packet);

                if (!currentAction&&timer.milliseconds()>6000) {
                    state = RedClose.ShootStates.CYCLE_1;
                    timer.reset();
                    timer.startTime();
                    robot.outtake.setLinkage(0.92);
                    robot.intake.setPower(0);

                }
                break;
            case CYCLE_1:
                robot.outtake.setPower(-0.5);
                robot.intake.setPower(0.8);
                robot.outtake.linkage.setPosition(0.92);
                currentAction = toSpike1.run(packet);

                if (!currentAction) {
                    state = RedClose.ShootStates.SHOOT_1;

                    timer.reset();
                    timer.startTime();
                }
                break;
            case SHOOT_1:
                if(timer.milliseconds()<600){
                    robot.outtake.setPower(-0.6);
                    robot.intake.setPower(-0.2);
                }
                if(timer.milliseconds() < 2000) {
                    robot.outtake.spinToRpm(3000);
                }

                else if(timer.milliseconds() < 2500) {
                    robot.outtake.setLinkage(0.5);
                    robot.intake.setPower(0);
                }


                else if(timer.milliseconds() < 3500){
                    robot.outtake.setLinkage(0.5);
                    robot.intake.setPower(0.6);
                }

                else if(timer.milliseconds()<4000){
                    robot.intake.setPower(0);
                    robot.outtake.spinToRpm(2000);
                }else{
                    robot.intake.setPower(0.8);
                }
                currentAction = toShootFrom1.run(packet);

                if (!currentAction&&timer.milliseconds()>5000) {
                    state = RedClose.ShootStates.LEAVE;
                    timer.reset();
                    timer.startTime();
                }
                break;
            case LEAVE:
                currentAction = leave.run(packet);

                if (!currentAction) {
                    state = RedClose.ShootStates.END;
                }
                break;
            case END:
                break;
        }
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}
