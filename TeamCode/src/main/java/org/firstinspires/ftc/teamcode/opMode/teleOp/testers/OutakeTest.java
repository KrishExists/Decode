//package org.firstinspires.ftc.teamcode.opMode.teleOp.testers;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.arcrobotics.ftclib.controller.PIDController;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.util.Range;
//import org.firstinspires.ftc.teamcode.subsystem.Robot;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//
//
//@Config
//@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "OuttakeTest", group = "")
//public class OutakeTest extends LinearOpMode {
//
//    public DcMotorEx outtake;
//    public DcMotorEx outtake2;
//    public Servo linkage;
//
//    // -------------------------
//    // Dashboard-Adjustable Constants
//    // -------------------------
//    public static double kp = 0.00625;
//    public static double ki = 0;
//    public static double kd = 0.001;
//
//    public static double rpmThresh = 50;
//    public static double targetRpm = 2000;
//
//    public PIDController rpmPID = new PIDController(kp, ki, kd);
//    private final ElapsedTime timer = new ElapsedTime();
//
//    // Dashboard
//    FtcDashboard dashboard = FtcDashboard.getInstance();
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//
//        outtake = hardwareMap.get(DcMotorEx.class, "Outtake");
//        outtake2 = hardwareMap.get(DcMotorEx.class, "Outtake2");
//
//        Robot robot = new Robot(hardwareMap, telemetry, new Pose2d(3,3,2));
//
//        outtake.setDirection(DcMotorEx.Direction.FORWARD);
//        outtake2.setDirection(DcMotorEx.Direction.REVERSE);
//
//        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        outtake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//
//        outtake.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        outtake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//
//        linkage = hardwareMap.get(Servo.class, "Linkage");
//
//        waitForStart();
//        if (isStopRequested()) return;
//
//        while (opModeIsActive()) {
//
//            robot.outtake.spinToRpm(targetRpm);
//
//            // -------------------------
//            // Normal Telemetry
//            // -------------------------
//            telemetry.addData("Velocity (ticks/s)", getVelocity());
//            telemetry.addData("Current RPM", currentRPM());
//            telemetry.addData("At Target?", upToRpm(targetRpm));
//            telemetry.update();
//
//            // -------------------------
//            // FTC DASHBOARD TELEMETRY
//
//
//            sleep(20);
//        }
//    }
//
//    // -------------------------
//    // PID RPM CONTROL
//    // -------------------------
//    public void spinToRpm(double rpm) {
//        rpmPID.setPID(kp, ki, kd);
//        double pidOutput = rpmPID.calculate(currentRPM(), rpm);
//        pidOutput = Range.clip(pidOutput, 0, 1);
//
//        outtake.setPower(pidOutput);
//        outtake2.setPower(pidOutput);
//
//        // Dashboard PID debugging
//        telemetry.addData("PID Output", pidOutput);
//        telemetry.addData("Error", rpm - currentRPM());
//    }
//
//    public boolean upToRpm(double rpm) {
//        double curr = currentRPM();
//        return (curr > rpm - rpmThresh) && (curr < rpm + rpmThresh);
//    }
//
//    public double currentRPM() {
//        return outtake.getVelocity() * 2.2; // conversion constant
//    }
//
//    public double getVelocity() {
//        return outtake.getVelocity();
//    }
//
//    public void setPower(double power) {
//        outtake.setPower(power);
//    }
//
//    public void setLinkage(double pos) {
//        linkage.setPosition(pos);
//    }
//}
