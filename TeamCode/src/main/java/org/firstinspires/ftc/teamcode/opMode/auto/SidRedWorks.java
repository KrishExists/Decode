package org.firstinspires.ftc.teamcode.opMode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseMap;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystem.Robot;

@Autonomous
public class SidRedWorks extends LinearOpMode {
    private final ElapsedTime timer = new ElapsedTime();

    final Pose2d START_POSE = new Pose2d(-51.5, 51.5, Math.toRadians(-144));
    final Pose2d LSHOOT = new Pose2d(-18, 18, Math.toRadians(-225));
    final Pose2d SPIKE1 = new Pose2d(36, 21, Math.toRadians(-270));
    final Pose2d SPIKE2 = new Pose2d(12, 44, Math.toRadians(-270));
    final Pose2d SPIKE3 = new Pose2d(-12, 44, Math.toRadians(-270));

    final PoseMap poseMap = pose -> new Pose2dDual<>(pose.position.x, pose.position.y.unaryMinus(), pose.heading.inverse());

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
        timer.startTime();
        while(opModeIsActive()) {
            if (isStopRequested()) return;

            robot.update(gamepad1, gamepad2);
            update();

            telemetry.addData("State", state);
            telemetry.addData("Outtake", robot.outtake.getVelocity());
            telemetry.addData("Timer",timer.milliseconds());
            // telemetry.addData("Current Action" , currentAction);
            telemetry.update();
        }
    }

    public void build_paths() {
        TrajectoryActionBuilder shootPrePath = robot.drive.drive.actionBuilder(START_POSE, poseMap)
                .strafeToLinearHeading(LSHOOT.position, LSHOOT.heading);

        TrajectoryActionBuilder toSpike3Path = shootPrePath.fresh()
                .turnTo(Math.toRadians(180))
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(SPIKE3, Math.toRadians(270));

        TrajectoryActionBuilder toShootFrom3Path = toSpike3Path.fresh()
                .strafeToLinearHeading(LSHOOT.position, LSHOOT.heading);

        TrajectoryActionBuilder toSpike2Path = toShootFrom3Path.fresh()
                .setTangent(Math.toRadians(340))
                .splineToSplineHeading(new Pose2d(4, -26, Math.toRadians(270)), Math.toRadians(340))
                .splineToLinearHeading(SPIKE2, Math.toRadians(270));

        TrajectoryActionBuilder toShootFrom2Path = toSpike2Path.fresh()
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(LSHOOT.position, LSHOOT.heading), Math.toRadians(180));

        TrajectoryActionBuilder toSpike1Path = toShootFrom2Path.fresh()
                .setTangent(Math.toRadians(345))
                .splineToSplineHeading(new Pose2d(27, -27, Math.toRadians(270)), Math.toRadians(345))
                .splineToLinearHeading(SPIKE1, Math.toRadians(270));

        TrajectoryActionBuilder toShootFrom1Path = toSpike1Path.fresh()
                .strafeToLinearHeading(LSHOOT.position, LSHOOT.heading);

        TrajectoryActionBuilder leavePath = toShootFrom1Path.fresh()
                .strafeTo(new Vector2d(0, -18));

        shootPre = shootPrePath.build();
        toSpike3 = toSpike3Path.build();
        toShootFrom3 = toShootFrom3Path.build();
        toSpike2 = toSpike2Path.build();
        toShootFrom2 = toShootFrom2Path.build();
        toSpike1 = toSpike1Path.build();
        toShootFrom1 = toShootFrom1Path.build();
        leave = leavePath.build();
    }

    public void update() {
        TelemetryPacket packet = new TelemetryPacket();
        // TODO: add robot actuations in state machine
        //timer.reset();

        switch (state) {
            case PRELOAD:
                if(timer.milliseconds() < 3000) {
                    robot.outtake.spinToRpm(2500);
                }

                else if(timer.milliseconds() < 3500) {
                    robot.outtake.setLinkage(0.6);
                    robot.intake.setPower(0);
                }


                else if(timer.milliseconds() < 4500){
                    robot.outtake.setLinkage(0.6);
                    robot.intake.setPower(0.6);
                }

                else if(timer.milliseconds()<5500){
                    robot.intake.setPower(0);
                    robot.outtake.spinToRpm(2500);
                }else{
                    robot.intake.setPower(0.8);
                }

                currentAction = shootPre.run(packet);

                if (!currentAction && timer.milliseconds() > 6000) {
                    state = SidRedWorks.ShootStates.CYCLE_3;
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
                    state = SidRedWorks.ShootStates.SHOOT_3;
                    timer.reset();
                    timer.startTime();
                }
                break;
            case SHOOT_3:
                if(timer.milliseconds()<1000){
                    robot.outtake.setPower(-1);
                    robot.intake.setPower(-0.6);
                }
                if(timer.milliseconds() < 2000) {
                    robot.outtake.spinToRpm(2500);
                }

                else if(timer.milliseconds() < 2500) {
                    robot.outtake.setLinkage(0.6);
                    robot.intake.setPower(0);
                }


                else if(timer.milliseconds() < 3500){
                    robot.intake.setPower(0.6);
                }

                else if(timer.milliseconds()<4000){
                    robot.intake.setPower(0);
                    robot.outtake.spinToRpm(2500);
                }else{
                    robot.intake.setPower(0.8);
                }
                currentAction = toShootFrom3.run(packet);

                if (!currentAction&&timer.milliseconds()>6000) {
                    state = SidRedWorks.ShootStates.CYCLE_2;
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
                    state = SidRedWorks.ShootStates.SHOOT_2;
                    timer.reset();
                    timer.startTime();

                }
                break;
            case SHOOT_2:
                if(timer.milliseconds()<1000){
                    robot.outtake.setPower(-0.6);
                    robot.intake.setPower(-0.2);
                }
                if(timer.milliseconds() < 2000) {
                    robot.outtake.spinToRpm(2500);
                }

                else if(timer.milliseconds() < 2500) {
                    robot.outtake.setLinkage(0.6);
                    robot.intake.setPower(0);
                }


                else if(timer.milliseconds() < 3500){
                    robot.outtake.setLinkage(0.6);
                    robot.intake.setPower(0.6);
                }

                else if(timer.milliseconds()<4000){
                    robot.intake.setPower(0);
                    robot.outtake.spinToRpm(2500);
                }else{
                    robot.intake.setPower(0.8);
                }
                currentAction = toShootFrom2.run(packet);

                if (!currentAction&&timer.milliseconds()>6000) {
                    state = SidRedWorks.ShootStates.CYCLE_1;
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
                    state = SidRedWorks.ShootStates.SHOOT_1;

                    timer.reset();
                    timer.startTime();
                }
                break;
            case SHOOT_1:
                if(timer.milliseconds()<1000){
                    robot.outtake.setPower(-0.6);
                    robot.intake.setPower(-0.2);
                }
                if(timer.milliseconds() < 2000) {
                    robot.outtake.spinToRpm(2500);
                }

                else if(timer.milliseconds() < 2500) {
                    robot.outtake.setLinkage(0.6);
                    robot.intake.setPower(0);
                }


                else if(timer.milliseconds() < 3500){
                    robot.outtake.setLinkage(0.6);
                    robot.intake.setPower(0.6);
                }

                else if(timer.milliseconds()<4000){
                    robot.intake.setPower(0);
                    robot.outtake.spinToRpm(2500);
                }else{
                    robot.intake.setPower(0.8);
                }
                currentAction = toShootFrom1.run(packet);

                if (!currentAction&&timer.milliseconds()>5000) {
                    state = SidRedWorks.ShootStates.LEAVE;
                    timer.reset();
                    timer.startTime();
                }
                break;
            case LEAVE:
                currentAction = leave.run(packet);

                if (!currentAction) {
                    state = SidRedWorks.ShootStates.END;
                }
                break;
            case END:
                break;
        }
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }


}
