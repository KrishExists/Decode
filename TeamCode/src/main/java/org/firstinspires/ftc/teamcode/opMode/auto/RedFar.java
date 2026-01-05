//package org.firstinspires.ftc.teamcode.opMode.auto;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
//import org.firstinspires.ftc.teamcode.roadRunner.MecanumDrive;
//import org.firstinspires.ftc.teamcode.subsystem.Robot;
//
//@Autonomous
//public class RedFar extends LinearOpMode {
//
//    private final ElapsedTime timer = new ElapsedTime();
//
//    // ---------------- POSES FROM MEEPMEEP ----------------
//    final Pose2d START_POSE = new Pose2d(60, 0, Math.toRadians(180));
//    final Pose2d LSHOOT  = new Pose2d(57, 3, Math.toRadians(160));
//    final Pose2d SPIKE1 = new Pose2d(59, 55, Math.toRadians(-270));
//    final Pose2d SPIKE2 = new Pose2d(59, 55, Math.toRadians(-270));
//    final Pose2d SPIKE3 = new Pose2d(55, 55, Math.toRadians(-270));
//
//    MecanumDrive drive;
//
//    Action shootPre, toSpike3, toShootFrom3, toSpike2, toShootFrom2,
//            toSpike1, toShootFrom1, leave;
//
//    boolean currentAction = true;
//
//    public enum ShootStates {
//        PRELOAD,
//        CYCLE_3,
//        SHOOT_3,
//        CYCLE_2,
//        SHOOT_2,
//        CYCLE_1,
//        SHOOT_1,
//        LEAVE,
//        END
//    }
//
//    ShootStates state = ShootStates.PRELOAD;
//
//    DcMotorEx transfer;
//    Robot robot;
//
//    int count = 0;
//    boolean wasPassedThresh = false;
//
//    @Override
//    public void runOpMode() {
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//
//        drive = new MecanumDrive(hardwareMap, START_POSE);
//        transfer = hardwareMap.get(DcMotorEx.class, "Transfer");
//        robot = new Robot(hardwareMap, telemetry, START_POSE);
//
//        buildPaths();
//
//        waitForStart();
//        timer.reset();
//
//        while (opModeIsActive()) {
//            update();
//            drive.updatePoseEstimate();
//
//            telemetry.addData("State", state);
//            telemetry.addData("RPM", robot.outtake.currentRPM());
//            telemetry.update();
//        }
//    }
//
//    // ---------------- PATHS (MATCH MEEPMEEP) ----------------
//    public void buildPaths() {
//
//        // PRELOAD → LSHOOT
//        shootPre = robot.drive.drive.actionBuilder(START_POSE)
//                .strafeToLinearHeading(LSHOOT.position, LSHOOT.heading)
//                .build();
//
//        // LSHOOT → SPIKE 3
//        toSpike3 = robot.drive.drive.actionBuilder(LSHOOT)
//                //.setTangent(Math.toRadians(90))
//                //.splineToLinearHeading(new Pose2d(SPIKE3.position.minus(new Vector2d(0,10)), Math.toRadians(70)), Math.toRadians(70))
//               // .setTangent(Math.toRadians(70))
//              .splineToSplineHeading(SPIKE3, SPIKE3.heading)
//                .strafeToLinearHeading(SPIKE1.position, SPIKE1.heading)
//                .strafeToLinearHeading(SPIKE3.position, SPIKE3.heading)
//                //.strafeToLinearHeading(SPIKE3.position, SPIKE3.heading)
//                .build();
//
//        // SPIKE 3 → LSHOOT
//        toShootFrom3 = robot.drive.drive.actionBuilder(SPIKE3)
//                .strafeToLinearHeading(LSHOOT.position, LSHOOT.heading)
//                .build();
//
//        // LSHOOT → SPIKE 2
//        toSpike2 = robot.drive.drive.actionBuilder(LSHOOT)
//                .strafeToLinearHeading(SPIKE2.position, Math.toRadians(-270))
//                .build();
//
//        // SPIKE 2 → LSHOOT
//        toShootFrom2 = robot.drive.drive.actionBuilder(SPIKE2)
//                .strafeToLinearHeading(LSHOOT.position, LSHOOT.heading)
//                .build();
//
//        // LSHOOT → SPIKE 1
//        toSpike1 = robot.drive.drive.actionBuilder(LSHOOT)
//                .strafeToLinearHeading(SPIKE1.position, Math.toRadians(-270))
//                .build();
//
//        // SPIKE 1 → LSHOOT
//        toShootFrom1 = robot.drive.drive.actionBuilder(SPIKE1)
//                .strafeToLinearHeading(LSHOOT.position, LSHOOT.heading)
//                .build();
//
//        // LEAVE
//        leave = robot.drive.drive.actionBuilder(LSHOOT)
//                .strafeTo(new Vector2d(48, 18))
//                .build();
//    }
//
//    // ---------------- STATE MACHINE ----------------
//    public void update() {
//        TelemetryPacket packet = new TelemetryPacket();
//
//        switch (state) {
//
//            case PRELOAD:
//                currentAction = shootPre.run(packet);
//
//                if(currentAction){
//                    robot.outtake.spinToRpm(3200);
//                    robot.outtake.linkage.setPosition(0.5);
//                    timer.reset();
//                    wasPassedThresh = false;
//                }else{
//                    if(timer.milliseconds()<0){
//                        telemetry.addData("Timer",timer.milliseconds());
//                        robot.outtake.linkage.setPosition(0.5);
//                        robot.outtake.spinToRpm(3200);
//                    }else{
//                        telemetry.addData("Count",count);
//                        robot.outtake.spinToRpm(3200);
//
//                        if(robot.outtake.currentRPM()>3100){
//                            telemetry.addData("Up to rpm",robot.outtake.currentRPM());
//                            robot.intake.setPower(1);
//                            transfer.setPower(-1);
//
//                            wasPassedThresh = true;
//
//                            if(timer.milliseconds()>8000){ // change back to 6500
//                                state = RedFar.ShootStates.CYCLE_3;
//                                transfer.setPower(-0.8);
//                            }
//                        }
//                        else{
//                            if(wasPassedThresh){
//                                count++;
//                                wasPassedThresh = false;
//                            }
//                            if(timer.milliseconds()>8000){ // change back to 6500
//                                state = RedFar.ShootStates.CYCLE_3;
//                                transfer.setPower(-0.8);
//                            }
//                            robot.intake.setPower(0);
//                            transfer.setPower(0);
//                        }
//                    }
//                }
//                break;
//            case CYCLE_3:
//                robot.outtake.setLinkage(0.5);
//                robot.outtake.setPower(-0.5);
//                robot.intake.setPower(0.8);
//                if (transfer.getCurrent(CurrentUnit.AMPS) >= 5.0) {
//                    transfer.setPower(0);
//                }
//
//                currentAction = toSpike3.run(packet);
//                if (!currentAction) {
//                    state = RedFar.ShootStates.SHOOT_3;
//                    timer.reset();
//                    timer.startTime();
//                    count = 0;
//                    wasPassedThresh = false;
//                }
//                break;
//
//            case SHOOT_3:
//                currentAction = toShootFrom3.run(packet);
//                //    state = NewRedTry.ShootStates.;
//
//                if(timer.milliseconds()<400&&timer.milliseconds()>200){
//                    transfer.setPower(0.5);
//                    robot.intake.setPower(1);
//                    robot.outtake.setPower(-0.5);
//                }
//                else if(timer.milliseconds()<400){
//                    robot.intake.setPower(0);
//                    robot.outtake.setPower(-0.5);
//                    transfer.setPower(0);
//
//                }
//                else if(currentAction){
//                    telemetry.addData("Timer",timer.milliseconds());
//                    robot.outtake.linkage.setPosition(0.5);
//                    robot.outtake.spinToRpm(3000);
//                    wasPassedThresh = false;
//                }
//                else{
//                    telemetry.addData("Count",count);
//                    robot.outtake.spinToRpm(3000);
//                    if(robot.outtake.currentRPM()>2993 || wasPassedThresh){
//                        telemetry.addData("Up to rpm",robot.outtake.currentRPM());
//                        robot.intake.setPower(1);
//                        transfer.setPower(-1);
//
//                        wasPassedThresh = true;
//
//                        if(count==3|| timer.milliseconds()>4000){
//                            state = RedFar.ShootStates.CYCLE_2;
//                        }
//                    }
//                    else{
//                        if(wasPassedThresh){
//                            count++;
//                            //wasPassedThresh = false;
//                        }
//                        if(count==3|| timer.milliseconds()>4500){
//                            state = RedFar.ShootStates.CYCLE_2;
//                        }
//                        robot.intake.setPower(0);
//                        transfer.setPower(0);
//                    }
//
//
//
//                }
//
//                break;
//
//            case CYCLE_2:
//                robot.outtake.setLinkage(0.5);
//                robot.outtake.setPower(-0.5);
//                robot.intake.setPower(0.8);
//                if (transfer.getCurrent(CurrentUnit.AMPS) >= 5.0) {
//                    transfer.setPower(0);
//                }
//
//                currentAction = toSpike2.run(packet);
//                if (!currentAction) {
//                    state = RedFar.ShootStates.SHOOT_2;
//                    timer.reset();
//                    timer.startTime();
//                    count = 0;
//                    wasPassedThresh = false;
//                }
//                break;
//
//            case SHOOT_2:
//                currentAction = toShootFrom2.run(packet);
//                //    state = NewRedTry.ShootStates.;
//
//                if(timer.milliseconds()<400&&timer.milliseconds()>200){
//                    transfer.setPower(0.5);
//                    robot.intake.setPower(1);
//                    robot.outtake.setPower(-0.5);
//                }
//                else if(timer.milliseconds()<420){
//                    robot.intake.setPower(1);
//                    robot.outtake.setPower(-0.5);
//                    transfer.setPower(0.5);
//
//                }
//                else if(currentAction){
//                    telemetry.addData("Timer",timer.milliseconds());
//                    robot.outtake.linkage.setPosition(0.5);
//                    robot.outtake.spinToRpm(2993);
//                    wasPassedThresh = false;
//
//                }
//                else{
//                    telemetry.addData("Count",count);
//                    robot.outtake.spinToRpm(2993);
//                    if(robot.outtake.currentRPM()>2901 || wasPassedThresh){
//                        telemetry.addData("Up to rpm",robot.outtake.currentRPM());
//                        robot.intake.setPower(1);
//                        transfer.setPower(-1);
//
//                        wasPassedThresh = true;
//
//                        if(timer.milliseconds()>6000){
//                            state = RedFar.ShootStates.CYCLE_1;
//                        }
//                    }
//                    else{
//                        if(wasPassedThresh){
//                            count++;
//                            // wasPassedThresh = false;
//                        }
//                        if(timer.milliseconds()>4500){
//                            state = RedFar.ShootStates.CYCLE_1;
//                        }
//                        robot.intake.setPower(0);
//                        transfer.setPower(0);
//                    }
//
//
//
//                }
//
//                break;
//
//            case CYCLE_1:
//                robot.outtake.setLinkage(0.5);
//                robot.outtake.setPower(-0.5);
//                robot.intake.setPower(0.8);
//                if (transfer.getCurrent(CurrentUnit.AMPS) >= 5.0) {
//                    transfer.setPower(0);
//                }
//
//                currentAction = toSpike1.run(packet);
//                if (!currentAction) {
//                    state = RedFar.ShootStates.SHOOT_1;
//                    timer.reset();
//                    timer.startTime();
//                    count = 0;
//                    wasPassedThresh = false;
//                }
//                break;
//
//            case SHOOT_1:
//                currentAction = toShootFrom1.run(packet);
//
//                if(timer.milliseconds()<400&&timer.milliseconds()>200){
//                    transfer.setPower(0.5);
//                    robot.intake.setPower(1);
//                    robot.outtake.setPower(-0.5);
//                }
//                else if(timer.milliseconds()<400){
//                    robot.intake.setPower(0);
//                    robot.outtake.setPower(-0.5);
//                    transfer.setPower(0);
//
//                }
//                else if(currentAction){
//                    telemetry.addData("Timer",timer.milliseconds());
//                    robot.outtake.linkage.setPosition(0.5);
//                    robot.outtake.spinToRpm(2993);
//                    wasPassedThresh = false;
//
//                }
//                else{
//                    telemetry.addData("Count",count);
//                    robot.outtake.spinToRpm(2993);
//                    if(robot.outtake.currentRPM()>2901 || wasPassedThresh){
//                        telemetry.addData("Up to rpm",robot.outtake.currentRPM());
//                        robot.intake.setPower(1);
//                        transfer.setPower(-1);
//
//                        wasPassedThresh = true;
//                        if(timer.milliseconds() > 4000)
//                            state = RedFar.ShootStates.LEAVE;
//                    }
//                    else{
//                        if(wasPassedThresh){
//                            count++;
//                            // wasPassedThresh = false;
//                        }
//                        if(count==20){
//                            state = RedFar.ShootStates.END;
//                        }
//                        robot.intake.setPower(0);
//                        transfer.setPower(0);
//                    }
//
//
//
//                }
//
//
//                break;
//
//            case LEAVE:
//                currentAction = leave.run(packet);
//                if (!currentAction) state = ShootStates.END;
//                break;
//
//            case END:
//                break;
//        }
//
//        FtcDashboard.getInstance().sendTelemetryPacket(packet);
//    }
//}
