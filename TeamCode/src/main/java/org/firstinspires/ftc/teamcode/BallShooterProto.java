package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class BallShooterProto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        DcMotorEx shooter1 = hardwareMap.get(DcMotorEx.class, "armMotorLeft");
        DcMotorEx shooter2 = hardwareMap.get(DcMotorEx.class, "armMotorRight");
       // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooter1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        double speed = 0.0;
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;
            if(gamepad1.dpadDownWasReleased()){
                if(speed<0.05){

                }else{
                    speed-=0.05;
                }
            }
            if(gamepad1.dpadUpWasReleased()){
                if(speed>0.096){

                }else{
                    speed+=0.05;
                }
            }
            if(gamepad1.dpadLeftWasReleased()){
                if(speed>0.91){

                }else{
                    speed +=0.1;
                }
            }
            if(gamepad1.rightBumperWasReleased()){
                speed +=0.1;
            }
            if(gamepad1.leftBumperWasReleased()){
                speed -=0.1;
            }
            if(gamepad1.dpadRightWasReleased()){
                if(speed<0.09){

                }else{
                    speed -=0.1;
                }
            }
            if(gamepad1.aWasReleased()){
                speed = 0.8;
            }
            if(gamepad1.bWasReleased()){
                speed = 0;
            }
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
            shooter2.setPower(speed);
            shooter1.setPower(speed);
            telemetry.addData("Speed: ", speed);
            telemetry.addData("Current Motor 1 Pos: ", (shooter1.getVelocity()/28) * 60);
            telemetry.addData("Current Motor 2 Pos: ", (shooter2.getVelocity()/28) *60);
            double motor1pos = shooter1.getVelocity()/28 * 60;
            double motor2pos = shooter2.getVelocity()/28 * 60;
            if(Math.abs(motor1pos-motor2pos)<50){
                telemetry.addData("Equal", "true");
            }else{
                telemetry.addData("Equal", "false");
            }
            telemetry.update();
        }
    }
}
