package org.firstinspires.ftc.teamcode.opMode.teleOp.testers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp(name = "YTester (Motor Power Dash)", group = "Testers")
public class YTester extends OpMode {

    // ===== Dashboard Tunables =====
    public static double frontLeftPower  = 0.0;
    public static double frontRightPower = 0.0;
    public static double backLeftPower   = 0.0;
    public static double backRightPower  = 0.0;

    // Optional: quick global scaling (keep at 1.0 normally)
    public static double scale = 1.0;

    // Optional: set motor directions here if you want
    public static boolean reverseFrontLeft  = false;
    public static boolean reverseFrontRight = false;
    public static boolean reverseBackLeft   = false;
    public static boolean reverseBackRight  = false;

    // ===== Motors =====
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Hardware Map names MUST match your config in RC phone
        frontLeftMotor  = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftMotor   = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor  = hardwareMap.get(DcMotor.class, "backRightMotor");

        // Set directions from dashboard toggles
        frontLeftMotor.setDirection(reverseFrontLeft ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        frontRightMotor.setDirection(reverseFrontRight ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(reverseBackLeft ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(reverseBackRight ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

        // Good defaults for testing
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("YTester ready. Open FTC Dashboard -> Config -> YTester");
        telemetry.update();
    }

    @Override
    public void loop() {
        // In case you flip reverse toggles live, keep applying them
        frontLeftMotor.setDirection(reverseFrontLeft ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        frontRightMotor.setDirection(reverseFrontRight ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(reverseBackLeft ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(reverseBackRight ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

        // Apply power from dashboard
        double fl = clip(frontLeftPower * scale);
        double fr = clip(frontRightPower * scale);
        double bl = clip(backLeftPower * scale);
        double br = clip(backRightPower * scale);

        frontLeftMotor.setPower(fl);
        frontRightMotor.setPower(fr);
        backLeftMotor.setPower(bl);
        backRightMotor.setPower(br);

        // Telemetry
        telemetry.addData("scale", scale);

        telemetry.addData("FL power cmd", fl);
        telemetry.addData("FR power cmd", fr);
        telemetry.addData("BL power cmd", bl);
        telemetry.addData("BR power cmd", br);

        telemetry.addData("FL encoder", frontLeftMotor.getCurrentPosition());
        telemetry.addData("FR encoder", frontRightMotor.getCurrentPosition());
        telemetry.addData("BL encoder", backLeftMotor.getCurrentPosition());
        telemetry.addData("BR encoder", backRightMotor.getCurrentPosition());

        telemetry.addData("FL dir", frontLeftMotor.getDirection());
        telemetry.addData("FR dir", frontRightMotor.getDirection());
        telemetry.addData("BL dir", backLeftMotor.getDirection());
        telemetry.addData("BR dir", backRightMotor.getDirection());

        telemetry.update();
    }

    @Override
    public void stop() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }

    private double clip(double v) {
        if (v > 1.0) return 1.0;
        if (v < -1.0) return -1.0;
        return v;
    }
}
