//package org.firstinspires.ftc.teamcode.opMode.auto;
//
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.subsystem.Robot;
//
//@Autonomous(name = "RedFar")
//public class RedFar extends LinearOpMode {
//
//    private Vector2d pos(double x, double y) { return new Vector2d(x, y); }
//    private double rad(double deg) { return Math.toRadians(deg); }
//
//    ElapsedTime timer = new ElapsedTime();
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        Pose2d START = new Pose2d(60, 20, rad(-180));
//
//        Robot robot = new Robot(hardwareMap, telemetry, START);
//        robot.init();
//
//        // ----------------------------------------------------
//        // BUILD MOTION PATH (EXACT MATCH TO YOUR MEEPMEEP)
//        // ----------------------------------------------------
//        TrajectoryActionBuilder path = robot.drive.drive.actionBuilder(START)
//
//                .strafeToLinearHeading(pos(54, 20), rad(-200))
//                .turnTo(rad(70))
//                .strafeToLinearHeading(pos(56, 60), rad(-270))
//                .strafeToLinearHeading(pos(54, 20), rad(-200))
//                .strafeToLinearHeading(pos(50, 20), rad(-200));
//
//        Action fullPath = path.build();
//
//        waitForStart();
//
//        timer.reset();
//
//        while (opModeIsActive()) {
//            robot.update();
//
//            // ----------------------------------------------------------
//            //                 INTAKE / OUTTAKE LOGIC
//            // ----------------------------------------------------------
//
//            double t = timer.milliseconds();
//
//            // 🔥 PRELOAD SHOOT (first 4.5 sec)
//            if (t < 1500) {
//                robot.outtake.spinToRpm(6000);   // spin up
//                robot.outtake.setLinkage(0.92);  // raised
//                robot.intake.setPower(0);
//            }
//            else if (t < 2500)xQ#@ {
//                robot.outtake.setLinkage(0.6);      // feed preload
//            }
//            else if (t < 3500) {
//                robot.intake.setPower(0);
//                robot.outtake.spinToRpm(6000);   // recover spin
//            }
//
//            // 📥 After 3.5 sec → RUN INTAKE
//            else if (t < 6000) {
//                robot.intake.setPower(0.8);
//                robot.outtake.setLinkage(0.6);  // up
//            }
//
//            // 📤 Endgame: stop intake & stop shooter safely
//            else {
//                robot.intake.setPower(0);
//                robot.outtake.setPower(0);
//            }
//
//            // ----------------------------------------------------------
//            //                    RUN PATH
//            // ----------------------------------------------------------
//            //boolean running = fullPath.run(robot.packet);
//            if (!true) break;
//
//            telemetry.addData("Time", t);
//            telemetry.update();
//        }
//    }
//}
