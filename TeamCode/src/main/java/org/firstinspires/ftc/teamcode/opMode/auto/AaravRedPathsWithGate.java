package org.firstinspires.ftc.teamcode.opMode.auto;

import androidx.core.os.TraceKt;

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
public class AaravRedPathsWithGate extends LinearOpMode {
    private final ElapsedTime timer = new ElapsedTime();

    Pose2d START_POSE = new Pose2d(-50.4, 50.2, Math.toRadians(-233));
    Pose2d LSHOOT = new Pose2d(-7, 3, Math.toRadians(-221));
    Pose2d SPIKE3 = new Pose2d(-12, 50, Math.toRadians(-270));
    Pose2d SPIKE2 = new Pose2d(12, 56, Math.toRadians(-275));
    Pose2d SPIKE1 = new Pose2d(35, 56, Math.toRadians(-270));

    Pose2d LEAVE = new Pose2d(9, 3, Math.toRadians(-227));

    public static Pose2d currentPose;

    public static boolean ran;
    MecanumDrive drive;

    Action shootPre, toSpike2, toShootFrom2, toGate, leaveGate, toGate2, leaveGate2, toSpike3, shootFrom3, toSpike1, shootFrom1, leave,openGate;

    boolean currentAction = true;
    public enum ShootStates {
        PRELOAD,
        CYCLE_2,
        SHOOT_2,
        CYCLE_3,
        SHOOT_3,
        CYCLE_1,
        SHOOT_1,
        Gate,
        LEAVE, END
    }

    ShootStates state = AaravRedPathsWithGate.ShootStates.PRELOAD;
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
            currentPose = drive.localizer.getPose();

            telemetry.addData("State", state);
            telemetry.addData("Outtake", robot.outtake.getVelocity());
            telemetry.addData("Timer", timer.milliseconds());
            telemetry.update();
        }
    }

    public void build_paths() {
        // PRELOAD → LSHOOT
        TrajectoryActionBuilder shootPrePath = robot.drive.drive.actionBuilder(START_POSE) // GOOD
                .strafeToLinearHeading(LSHOOT.position, LSHOOT.heading);


        // LSHOOT → SPIKE 2
        TrajectoryActionBuilder toSpike2Path = robot.drive.drive.actionBuilder(LSHOOT) // GOOD
                .setTangent(Math.toRadians(20))
                .strafeToLinearHeading(
                        new Pose2d(12, 10,Math.toRadians(-270)).position,
                        Math.toRadians(-270)
                )
                .strafeToLinearHeading(SPIKE2.position, Math.toRadians(-270));
        TrajectoryActionBuilder OpenGate = robot.drive.drive.actionBuilder(SPIKE2)
                .strafeToLinearHeading(new Vector2d(12,50),Math.toRadians(-270))
                .strafeToLinearHeading(new Vector2d(6,50),Math.toRadians(-270))
                .strafeToLinearHeading(new Vector2d(6,54),Math.toRadians(-270));

        // SPIKE 2 → LSHOOT
        TrajectoryActionBuilder toShootFrom2Path = robot.drive.drive.actionBuilder(SPIKE2) // GOOD
                .strafeToLinearHeading(LSHOOT.position, LSHOOT.heading);



        //LSHOOT -> SPIKE3
        TrajectoryActionBuilder toSpike3Path = robot.drive.drive.actionBuilder(LSHOOT)
                .strafeToLinearHeading(new Pose2d(-12,3,5).position,SPIKE3.heading)
                .strafeToLinearHeading(SPIKE3.position, SPIKE3.heading);

        //SPIKE3 -> LSHOOT
        TrajectoryActionBuilder toShootFrom3Path = robot.drive.drive.actionBuilder(SPIKE3)
                .strafeToLinearHeading(LSHOOT.position, LSHOOT.heading);

        TrajectoryActionBuilder toSpike1Path = robot.drive.drive.actionBuilder(LSHOOT)
                .strafeToLinearHeading(new Vector2d(35,13), Math.toRadians(-270))
                .strafeToLinearHeading(SPIKE1.position, Math.toRadians(-270));


        TrajectoryActionBuilder toShootFrom1Path = robot.drive.drive.actionBuilder(SPIKE1)
                .strafeToLinearHeading(LSHOOT.position, LSHOOT.heading);

        TrajectoryActionBuilder toLeavePath = robot.drive.drive.actionBuilder(LSHOOT)
                .strafeToLinearHeading(LEAVE.position, LEAVE.heading);

        //HELPS ME REMEMBER HOW TO BUILD THE PATHS Action shootPre, toSpike2, toShootFrom2, toGate, leaveGate, toGate2, leaveGate2, toSpike3, shootFrom3, toSpike1, shootFrom1;

        shootPre = shootPrePath.build();
        toSpike2 = toSpike2Path.build();
        toShootFrom2 = toShootFrom2Path.build();
        toSpike3 = toSpike3Path.build();
        shootFrom3 = toShootFrom3Path.build();
        toSpike1 = toSpike1Path.build();
        shootFrom1 = toShootFrom1Path.build();
        leave = toLeavePath.build();
        openGate = OpenGate.build();

    }

    public void update() {
        TelemetryPacket packet = new TelemetryPacket();

        switch (state) {
            case PRELOAD:
                currentAction = shootPre.run(packet);

                if(currentAction){
                    robot.outtake.spinToRpm(2400);
                    robot.outtake.linkage.setPosition(0.5);
                    robot.intake.setPower(1);
                    transfer.setPower(1);
                    timer.reset();
                    wasPassedThresh = false;
                }else{
                    if(timer.milliseconds()<0){
                        telemetry.addData("Timer",timer.milliseconds());
                        robot.outtake.linkage.setPosition(0.5);
                        robot.outtake.spinToRpm(2450);
                        robot.intake.setPower(1);
                        transfer.setPower(1);
                    }else{
                        telemetry.addData("Count",count);
                        robot.outtake.spinToRpm(2450);

                        if(robot.outtake.currentRPM()>2350 || wasPassedThresh){
                            telemetry.addData("Up to rpm",robot.outtake.currentRPM());
                            robot.intake.setPower(1);
                            transfer.setPower(-1);

                            wasPassedThresh = true;

                            if(timer.milliseconds()>650){ // change back to 6500
                                state = AaravRedPathsWithGate.ShootStates.CYCLE_2;
                                transfer.setPower(-0.8);
                            }
                        }
                        else{
                            if(wasPassedThresh){
                                count++;
                                //wasPassedThresh = false;
                            }
                            if(timer.milliseconds()>2000){ // change back to 6500
                                state = AaravRedPathsWithGate.ShootStates.CYCLE_2;
                                transfer.setPower(-0.8);
                            }
                            robot.intake.setPower(0);
                            transfer.setPower(0);
                        }
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
                    state = ShootStates.Gate;
                    timer.reset();
                    timer.startTime();
                    count = 0;
                    wasPassedThresh = false;
                }
                break;
            case Gate:
                currentAction = openGate.run(packet);
                if(!currentAction){
                    state = ShootStates.SHOOT_2;
                    timer.reset();
                    timer.startTime();
                    count = 0;
                    wasPassedThresh = false;
                }
            case SHOOT_2:
                currentAction = toShootFrom2.run(packet);

                if(timer.milliseconds()<400&&timer.milliseconds()>200){
                    transfer.setPower(0.5);
                    robot.intake.setPower(1);
                    robot.outtake.setPower(-0.5);
                }
                else if(timer.milliseconds()<420){
                    robot.intake.setPower(1);
                    robot.outtake.setPower(-0.5);
                    transfer.setPower(0.5);

                }
                else if(currentAction){
                    telemetry.addData("Timer",timer.milliseconds());
                    robot.outtake.linkage.setPosition(0.5);
                    robot.outtake.spinToRpm(2450);
                    wasPassedThresh = false;

                }
                else{
                    telemetry.addData("Count",count);
                    robot.outtake.spinToRpm(2450);
                    if(robot.outtake.currentRPM()>2350 || wasPassedThresh){
                        telemetry.addData("Up to rpm",robot.outtake.currentRPM());
                        robot.intake.setPower(1);
                        transfer.setPower(-1);

                        wasPassedThresh = true;

                        if(timer.milliseconds()>3000){
                            state = ShootStates.CYCLE_3;
                        }
                    }
                    else{
                        if(wasPassedThresh){
                            count++;
                            // wasPassedThresh = false;
                        }
                        if(timer.milliseconds()>4500){
                            state = ShootStates.CYCLE_3;
                        }
                        robot.intake.setPower(0);
                        transfer.setPower(0);
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
                    state = AaravRedPathsWithGate.ShootStates.SHOOT_3;
                    timer.reset();
                    timer.startTime();
                    count = 0;
                    wasPassedThresh = false;
                }
                break;
            case SHOOT_3:
                currentAction = shootFrom3.run(packet);
                //    state = NewRedTry.ShootStates.;

                if(timer.milliseconds()<400&&timer.milliseconds()>200){
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
                    robot.outtake.spinToRpm(2450);
                    wasPassedThresh = false;
                }
                else{
                    telemetry.addData("Count",count);
                    robot.outtake.spinToRpm(25450);
                    if(robot.outtake.currentRPM()>2350 || wasPassedThresh){
                        telemetry.addData("Up to rpm",robot.outtake.currentRPM());
                        robot.intake.setPower(1);
                        transfer.setPower(-1);

                        wasPassedThresh = true;

                        if(count==3|| timer.milliseconds()>4000){
                            state = ShootStates.CYCLE_1;
                        }
                    }
                    else{
                        if(wasPassedThresh){
                            count++;
                            //wasPassedThresh = false;
                        }
                        if(count==3|| timer.milliseconds()>4500){
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
                    state = AaravRedPathsWithGate.ShootStates.SHOOT_1;
                    timer.reset();
                    timer.startTime();
                    count = 0;
                    wasPassedThresh = false;
                }
                break;
            case SHOOT_1:
                currentAction = shootFrom1.run(packet);

                if(timer.milliseconds()<400&&timer.milliseconds()>200){
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
                    robot.outtake.spinToRpm(2450);
                    wasPassedThresh = false;

                }
                else{
                    telemetry.addData("Count",count);
                    robot.outtake.spinToRpm(2450);
                    if(robot.outtake.currentRPM()>2350 || wasPassedThresh){
                        telemetry.addData("Up to rpm",robot.outtake.currentRPM());
                        robot.intake.setPower(1);
                        transfer.setPower(-1);

                        wasPassedThresh = true;
                        if(timer.milliseconds() > 4000)
                            state = AaravRedPathsWithGate.ShootStates.LEAVE;
                    }
                    else{
                        if(wasPassedThresh){
                            count++;
                            // wasPassedThresh = false;
                        }
                        if(count==3){
                            state = AaravRedPathsWithGate.ShootStates.LEAVE;
                        }
                        robot.intake.setPower(0);
                        transfer.setPower(0);
                    }



                }

                break;

            case LEAVE:
                currentAction = leave.run(packet);

                if (!currentAction) {
                    state = AaravRedPathsWithGate.ShootStates.END;
                }
                break;
            case END:
                break;
        }
    }
}
