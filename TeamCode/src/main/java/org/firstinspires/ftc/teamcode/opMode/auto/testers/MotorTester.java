package org.firstinspires.ftc.teamcode.opMode.auto.testers;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.util.TeamConstants;
@TeleOp(name = "Motor Tester", group = "P")
@Config
public class MotorTester extends LinearOpMode {

    DcMotor motor;

    public static String hwName;

    public static double power;

    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotor.class, hwName);
        waitForStart();
        while (opModeIsActive()){
            motor.setPower(power);
        }
    }

}
