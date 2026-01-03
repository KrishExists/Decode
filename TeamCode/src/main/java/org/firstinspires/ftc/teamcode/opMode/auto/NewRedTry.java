package org.firstinspires.ftc.teamcode.opMode.auto;

import android.graphics.drawable.ScaleDrawable;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.roadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.Robot;

@Autonomous
public class NewRedTry extends LinearOpMode {
    private final ElapsedTime timer = new ElapsedTime();

    // ---- UPDATED TO MATCH YOUR MEEPMEEP FILE EXACTLY ----
    final Pose2d START_POSE = new Pose2d(-49.2, 50.1, Math.toRadians(-144));
    final Pose2d LSHOOT = new Pose2d(-22, 3, Math.toRadians(-235));
    final Pose2d SPIKE3 = new Pose2d(-8, 52, Math.toRadians(-270));
    final Pose2d SPIKE1Init = new Pose2d(40, 10, Math.toRadians(-270));
    final Pose2d SPIKE2 = new Pose2d(16, 57, Math.toRadians(-270));
    final Pose2d SPIKE1 = new Pose2d(40, 51, Math.toRadians(-270));


    MecanumDrive drive;

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
    DcMotorEx transfer;

    Robot robot;
    int count =0;
    boolean wasPassedThresh = false;
    @Override
    public void runOpMode() throws InterruptedException {
        timer.reset();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new MecanumDrive(hardwareMap, START_POSE);
        transfer = hardwareMap.get(DcMotorEx.class, "Transfer");


        robot = new Robot(hardwareMap, telemetry, START_POSE);

        build_paths();

        waitForStart();
        timer.reset();

        while (opModeIsActive()) {
            if (isStopRequested()) return;


            update();
            drive.updatePoseEstimate();
            Pose2d currentPose = drive.localizer.getPose();

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
        //LShoot -> GOFORWARD


        // LSHOOT → SPIKE 3
        TrajectoryActionBuilder toSpike3Path = robot.drive.drive.actionBuilder(LSHOOT)
                .turnTo(Math.toRadians(-270))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(SPIKE3, Math.toRadians(90));

        // SPIKE 3 → LSHOOT
        TrajectoryActionBuilder toShootFrom3Path = robot.drive.drive.actionBuilder(SPIKE3)
                .strafeToLinearHeading(LSHOOT.position, LSHOOT.heading);

        // LSHOOT → SPIKE 2
        TrajectoryActionBuilder toSpike2Path = robot.drive.drive.actionBuilder(LSHOOT)
                .setTangent(Math.toRadians(20))
                .strafeToLinearHeading(
                        new Pose2d(18, 10,Math.toRadians(-270)).position,
                        Math.toRadians(-270)
                )
                .strafeToLinearHeading(SPIKE2.position, Math.toRadians(-270));

        // SPIKE 2 → LSHOOT
        TrajectoryActionBuilder toShootFrom2Path = robot.drive.drive.actionBuilder(SPIKE2)
                .setTangent(Math.toRadians(20))
                .strafeToLinearHeading(
                        new Pose2d(14, 10,Math.toRadians(-270)).position,
                        Math.toRadians(-270)
                )
                .strafeToLinearHeading(LSHOOT.position, LSHOOT.heading);

        // LSHOOT → SPIKE 1
        TrajectoryActionBuilder toSpike1Path = robot.drive.drive.actionBuilder(LSHOOT)
                .splineToLinearHeading(SPIKE1Init, SPIKE1Init.heading)
                .strafeToLinearHeading(SPIKE1.position, SPIKE1.heading);

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
        // TODO: Fix the robot cycle between at shoot3 as the logic there is very incorrect.
        //timer.reset();

        switch (state) {
            case PRELOAD:

//                if(timer.milliseconds() < 3000) {
//                    robot.outtake.setVelocity(1750);
//                }
//
//                else if(timer.milliseconds() < 3500) {
//                    robot.outtake.setLinkage(0.6);
//                    robot.intake.setPower(0);
//                }
//
//
//                else if(timer.milliseconds() < 4500){
//                    robot.outtake.setLinkage(0.6);
//                    robot.intake.setPower(0.6);
//                }
//
//                else if(timer.milliseconds()<5500){
//                    robot.intake.setPower(0);
//                    robot.outtake.setVelocity(1750);
//                }else{
//                    robot.intake.setPower(0.8);
//                }
//
//
//                if (!currentAction && timer.milliseconds() > 8000) {
//                    state = NewRedTry.ShootStates.CYCLE_3;
//                    timer.reset();
//                    timer.startTime();
//                    robot.outtake.setLinkage(0.92);
//                    robot.intake.setPower(0);
//                }


                currentAction = shootPre.run(packet);

                if(currentAction){
                    robot.outtake.spinToRpm(2400);
                    robot.outtake.linkage.setPosition(0.5);
                    timer.reset();
                    wasPassedThresh = false;
                }else{
                    if(timer.milliseconds()<0){
                        telemetry.addData("Timer",timer.milliseconds());
                        robot.outtake.linkage.setPosition(0.5);
                        robot.outtake.spinToRpm(2400);
                    }else{
                        telemetry.addData("Count",count);
                        robot.outtake.spinToRpm(2500);

                        if(robot.outtake.currentRPM()>2300 || wasPassedThresh){
                            telemetry.addData("Up to rpm",robot.outtake.currentRPM());
                            robot.intake.setPower(1);
                            transfer.setPower(-1);

                            wasPassedThresh = true;

                            if(timer.milliseconds()>2000){ // change back to 6500
                                state = ShootStates.CYCLE_3;
                                transfer.setPower(-0.8);
                            }
                        }
                        else{
                            if(wasPassedThresh){
                                count++;
                                //wasPassedThresh = false;
                            }
                            if(timer.milliseconds()>2000){ // change back to 6500
                                state = ShootStates.CYCLE_3;
                                transfer.setPower(-0.8);
                            }
                            robot.intake.setPower(0);
                            transfer.setPower(0);
                        }
                    }
                }
                break;
            case CYCLE_3:
                robot.outtake.setLinkage(0.5);
                robot.outtake.setPower(-0.5);
                robot.intake.setPower(0.8);
                if (transfer.getCurrent(CurrentUnit.AMPS) >= 5.0) {
                    transfer.setPower(0);
                }

                currentAction = toSpike3.run(packet);
                if (!currentAction) {
                    state = NewRedTry.ShootStates.SHOOT_3;
                    timer.reset();
                    timer.startTime();
                    count = 0;
                    wasPassedThresh = false;
                }
                break;
            case SHOOT_3:
                currentAction = toShootFrom3.run(packet);
                //    state = NewRedTry.ShootStates.;

                if(timer.milliseconds()<400&&timer.milliseconds()>0){
                    transfer.setPower(0.5);
                    robot.intake.setPower(1);
                    robot.outtake.setPower(-0.5);
                }
                else if(timer.milliseconds()<400){
                    robot.intake.setPower(0);
                    robot.outtake.setPower(-0.5);
                    transfer.setPower(0);

                }
                else if(currentAction){
                    telemetry.addData("Timer",timer.milliseconds());
                    robot.outtake.linkage.setPosition(0.5);
                    robot.outtake.spinToRpm(2400);
                    wasPassedThresh = false;
                }
                else{
                    telemetry.addData("Count",count);
                    robot.outtake.spinToRpm(2400);
                    if(robot.outtake.currentRPM()>2300 || wasPassedThresh){
                        telemetry.addData("Up to rpm",robot.outtake.currentRPM());
                        robot.intake.setPower(1);
                        transfer.setPower(-1);

                        wasPassedThresh = true;

                        if(count==3|| timer.milliseconds()>4000){
                            state = ShootStates.CYCLE_2;
                        }
                    }
                    else{
                        if(wasPassedThresh){
                            count++;
                            //wasPassedThresh = false;
                        }
                        if(count==3|| timer.milliseconds()>4500){
                            state = ShootStates.CYCLE_2;
                        }
                        robot.intake.setPower(0);
                        transfer.setPower(0);
                    }



                }

                break;
            case CYCLE_2:
                robot.outtake.setLinkage(0.5);
                robot.outtake.setPower(-0.5);
                robot.intake.setPower(0.8);
                if (transfer.getCurrent(CurrentUnit.AMPS) >= 5.0) {
                    transfer.setPower(0);
                }

                currentAction = toSpike2.run(packet);
                if (!currentAction) {
                    state = NewRedTry.ShootStates.SHOOT_2;
                    timer.reset();
                    timer.startTime();
                    count = 0;
                    wasPassedThresh = false;
                }
                break;
            case SHOOT_2:
                currentAction = toShootFrom2.run(packet);
                //    state = NewRedTry.ShootStates.;

                if(timer.milliseconds()<400&&timer.milliseconds()>0){
                    transfer.setPower(0.5);
                    robot.intake.setPower(1);
                    robot.outtake.setPower(-0.5);
                }
                else if(timer.milliseconds()<420){
                    robot.intake.setPower(0);
                    robot.outtake.setPower(-0.5);
                    transfer.setPower(0);

                }
                else if(currentAction){
                    telemetry.addData("Timer",timer.milliseconds());
                    robot.outtake.linkage.setPosition(0.5);
                    robot.outtake.spinToRpm(2400);
                    wasPassedThresh = false;

                }
                else{
                    telemetry.addData("Count",count);
                    robot.outtake.spinToRpm(2400);
                    if(robot.outtake.currentRPM()>2300 || wasPassedThresh){
                        telemetry.addData("Up to rpm",robot.outtake.currentRPM());
                        robot.intake.setPower(1);
                        transfer.setPower(-1);

                        wasPassedThresh = true;

                        if(timer.milliseconds()>6000){
                            state = ShootStates.CYCLE_1;
                        }
                    }
                    else{
                        if(wasPassedThresh){
                            count++;
                           // wasPassedThresh = false;
                        }
                        if(timer.milliseconds()>4500){
                            state = ShootStates.CYCLE_1;
                        }
                        robot.intake.setPower(0);
                        transfer.setPower(0);
                    }



                }

                break;
            case CYCLE_1:
                robot.outtake.setLinkage(0.5);
                robot.outtake.setPower(-0.5);
                robot.intake.setPower(0.8);
                if (transfer.getCurrent(CurrentUnit.AMPS) >= 5.0) {
                    transfer.setPower(0);
                }

                currentAction = toSpike1.run(packet);
                if (!currentAction) {
                    state = ShootStates.SHOOT_1;
                    timer.reset();
                    timer.startTime();
                    count = 0;
                    wasPassedThresh = false;
                }
                break;
            case SHOOT_1:
                currentAction = toShootFrom1.run(packet);

                if(timer.milliseconds()<400&&timer.milliseconds()>0){
                    transfer.setPower(0.5);
                    robot.intake.setPower(1);
                    robot.outtake.setPower(-0.5);
                }
                else if(timer.milliseconds()<400){
                    robot.intake.setPower(0);
                    robot.outtake.setPower(-0.5);
                    transfer.setPower(0);

                }
                else if(currentAction){
                    telemetry.addData("Timer",timer.milliseconds());
                    robot.outtake.linkage.setPosition(0.5);
                    robot.outtake.spinToRpm(2400);
                    wasPassedThresh = false;

                }
                else{
                    telemetry.addData("Count",count);
                    robot.outtake.spinToRpm(2400);
                    if(robot.outtake.currentRPM()>2300){
                        telemetry.addData("Up to rpm",robot.outtake.currentRPM());
                        robot.intake.setPower(1);
                        transfer.setPower(-1);

                        wasPassedThresh = true;
                    }
                    else{
                        if(wasPassedThresh){
                            count++;
                            wasPassedThresh = false;
                        }
                        if(count==3){
                            state = ShootStates.END;
                        }
                        robot.intake.setPower(0);
                        transfer.setPower(0);
                    }



                }


                break;
            case LEAVE:
                currentAction = leave.run(packet);

                if (!currentAction) {
                    state = NewRedTry.ShootStates.END;
                }
                break;
            case END:
                break;
        }
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}