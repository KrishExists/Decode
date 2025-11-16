package org.firstinspires.ftc.teamcode.control;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystem.Robot;

public class DriveControl implements Control {

    private Drivetrain drive;
    private Gamepad gp1, gp2;

    public DriveControl(Drivetrain d, Gamepad gp1, Gamepad gp2) {
        this.drive = d;
        this.gp1 = gp1;
        this.gp2 = gp2;
    }

    public DriveControl(Robot robot, Gamepad gp1, Gamepad gp2) {
        this(robot.drive, gp1, gp2);
    }

    @Override
    public void update() {
        boolean bumpers = gp1.right_bumper || gp1.left_bumper;

        PoseVelocity2d driveInput = new PoseVelocity2d(
                new Vector2d(
                        -gp1.left_stick_y * (bumpers ? 0.5 : 1),
                        -gp1.left_stick_x * (bumpers ? 0.5 : 1)
                ),
                -gp1.right_stick_x * (bumpers ? 0.5 : 1)
        );

        drive.drive.setDrivePowers(driveInput);
    }
}
