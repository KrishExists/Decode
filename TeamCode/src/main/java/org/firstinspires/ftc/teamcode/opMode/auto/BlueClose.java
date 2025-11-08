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

import org.firstinspires.ftc.teamcode.subsystem.Robot;

@Autonomous
public class BlueClose extends LinearOpMode {

    final Pose2d START_POSE = new Pose2d(-51.5, -51.5, Math.toRadians(144));
    final Pose2d LSHOOT = new Pose2d(-18, -18, Math.toRadians(225));
    final Pose2d SPIKE1 = new Pose2d(36, -51, Math.toRadians(270));
    final Pose2d SPIKE2 = new Pose2d(12, -54, Math.toRadians(270));
    final Pose2d SPIKE3 = new Pose2d(-12, -54, Math.toRadians(270));

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
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot = new Robot(hardwareMap, telemetry, START_POSE);

        build_paths();

        robot.init();

        while(opModeIsActive()) {
            if (isStopRequested()) return;

            robot.update();
            update();

            telemetry.addData("State", state);
            telemetry.update();
        }
    }

    public void build_paths() {
        TrajectoryActionBuilder shootPrePath = robot.drive.drive.actionBuilder(START_POSE)
                .strafeToLinearHeading(LSHOOT.position, LSHOOT.heading);

        TrajectoryActionBuilder toSpike3Path = shootPrePath.fresh()
                .turnTo(Math.toRadians(270))
                .setTangent(Math.toRadians(305))
                .splineToLinearHeading(SPIKE3, Math.toRadians(270));

        TrajectoryActionBuilder toShootFrom3Path = toSpike3Path.fresh()
                .strafeToLinearHeading(LSHOOT.position, LSHOOT.heading);

        TrajectoryActionBuilder toSpike2Path = toShootFrom3Path.fresh()
                .setTangent(Math.toRadians(340))
                .splineToSplineHeading(new Pose2d(4, -26, Math.toRadians(270)), Math.toRadians(340))
                .splineToLinearHeading(SPIKE2, Math.toRadians(270));

        TrajectoryActionBuilder toShootFrom2Path = toSpike2Path.fresh()
                .strafeToLinearHeading(LSHOOT.position, LSHOOT.heading);

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

        switch (state) {
            case PRELOAD:

                currentAction = shootPre.run(packet);

                if (!currentAction) {
                    state = ShootStates.CYCLE_3;
                }
                break;
            case CYCLE_3:
                currentAction = toSpike3.run(packet);
                if (!currentAction) {
                    state = ShootStates.SHOOT_3;
                }
                break;
            case SHOOT_3:
                currentAction = toShootFrom3.run(packet);

                if (!currentAction) {
                    state = ShootStates.CYCLE_2;
                }
                break;
            case CYCLE_2:
                currentAction = toSpike2.run(packet);

                if (!currentAction) {
                    state = ShootStates.SHOOT_2;
                }
                break;
            case SHOOT_2:
                currentAction = toShootFrom2.run(packet);

                if (!currentAction) {
                    state = ShootStates.CYCLE_1;
                }
                break;
            case CYCLE_1:
                currentAction = toSpike1.run(packet);

                if (!currentAction) {
                    state = ShootStates.SHOOT_1;
                }
                break;
            case SHOOT_1:
                currentAction = toShootFrom1.run(packet);

                if (!currentAction) {
                    state = ShootStates.LEAVE;
                }
                break;
            case LEAVE:
                currentAction = leave.run(packet);

                if (!currentAction) {
                    state = ShootStates.END;
                }
                break;
            case END:
                break;
        }
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

}
