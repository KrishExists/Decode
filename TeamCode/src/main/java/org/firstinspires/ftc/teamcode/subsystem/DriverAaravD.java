//package org.firstinspires.ftc.teamcode.subsystem;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Gamepad;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//
//public class DriverAaravD implements Subsystem {
//
//    public DcMotor lfMotor, lbMotor, rfMotor, rbMotor;
//
//    Telemetry telemetry;
//
//    double y = 0, x = 0, rx = 0, denominator = 0;
//    public double lfPower = 0, lbPower = 0, rfPower = 0, rbPower = 0;
//
//    public boolean bumpToggle = false;
//
//    public DriverAaravD (HardwareMap map, Telemetry telemetry) {
//        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        lfMotor = map.get(DcMotor.class, "frontLeftMotor");
//        lbMotor = map.get(DcMotor.class, "backLeftMotor");
//        rfMotor = map.get(DcMotor.class, "frontRightMotor");
//        rbMotor = map.get(DcMotor.class, "backRightMotor");
////
////        lfMotor.setDirection(DcMotor.Direction.REVERSE);
////        lbMotor.setDirection(DcMotor.Direction.REVERSE);
////        rfMotor.setDirection(DcMotor.Direction.REVERSE);
////        rbMotor.setDirection(DcMotor.Direction.FORWARD);
//
//        lfMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        lbMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rfMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rbMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//    }
//
//    @Override
//    public void init() {
//        telemetry.addData("Drivetrain", "Initialized");
//        telemetry.update();
//    }
//
//    public void setDriveVectors(double y, double x, double rx) {
//        this.y = y;
//        this.x = x;
//        this.rx = rx;
//    }
//
//    public void update(Gamepad gamepad1) {
//        double y = -gamepad1.left_stick_y;
//        double x = gamepad1.left_stick_x * 1.1;
//        double rx = gamepad1.right_stick_x;
//
//        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
//
//        lfMotor.setPower((y + x + rx) / denominator);
//        lbMotor.setPower((y - x + rx) / denominator);
//        rfMotor.setPower((y - x - rx) / denominator);
//        rbMotor.setPower((y + x - rx) / denominator);
//    }
//
//    public void updateCtrls(Gamepad gp1, Gamepad gp2) {
//
//        if (gp1.rightBumperWasPressed()) {
//            bumpToggle = !bumpToggle;
//        }
//
//        if (bumpToggle) {
//            setDriveVectors(-gp1.left_stick_y * 0.3, gp1.left_stick_x * 0.4, gp1.right_stick_x * 0.3);
//        } else {
//            setDriveVectors(-gp1.left_stick_y, gp1.left_stick_x, gp1.right_stick_x);
//        }
//
//        update(gp2);
//    }
//}
