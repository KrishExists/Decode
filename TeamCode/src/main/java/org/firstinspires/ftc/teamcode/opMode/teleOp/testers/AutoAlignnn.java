package org.firstinspires.ftc.teamcode.opMode.teleOp.testers;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystem.AprilTagAlignment;
import org.firstinspires.ftc.teamcode.subsystem.Drivetrain;

@TeleOp(name = "AprilTagAlignmentTele", group = " ")
public class AutoAlignnn extends LinearOpMode {
    public AprilTagAlignment aprilTagAlignment;

    public Drivetrain drivetrain;

    @Override
    public void runOpMode(){
        drivetrain.init();

        waitForStart();

        while (opModeIsActive()){
            drivetrain.update();

        }

    }


}
