package org.firstinspires.ftc.teamcode.opMode.teleOp.testers;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection;

import com.qualcomm.hardware.andymark.AndyMarkIMUOrientationOnRobot;
import com.qualcomm.hardware.andymark.AndyMarkIMUOrientationOnRobot.I2cPortFacingDirection;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "IMU Testers (Control Hub + Pinpoint)", group = "Testers")
public class IMUTesters extends LinearOpMode {

    private IMU controlHubIMU;
    private IMU pinpointIMU;

    @Override
    public void runOpMode() {

        /* -------------------- Control Hub IMU -------------------- */
        controlHubIMU = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot controlHubOrientation =
                new RevHubOrientationOnRobot(
                        LogoFacingDirection.RIGHT,
                        UsbFacingDirection.UP
                );

        controlHubIMU.initialize(new IMU.Parameters(controlHubOrientation));

        /* -------------------- Pinpoint (AndyMark) IMU -------------------- */
        pinpointIMU = hardwareMap.get(IMU.class, "pinIMU");

        AndyMarkIMUOrientationOnRobot pinpointOrientation =
                new AndyMarkIMUOrientationOnRobot(
                        AndyMarkIMUOrientationOnRobot.LogoFacingDirection.UP,
                        I2cPortFacingDirection.UP
                );

        pinpointIMU.initialize(new IMU.Parameters(pinpointOrientation));

        telemetry.addLine("IMUs Initialized");
        telemetry.addLine("Press PLAY");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            /* -------- Reset Yaw -------- */
            if (gamepad1.y) {
                controlHubIMU.resetYaw();
                pinpointIMU.resetYaw();
                telemetry.addLine("Yaw Reset");
            }

            /* -------- Control Hub Readings -------- */
            YawPitchRollAngles chAngles =
                    controlHubIMU.getRobotYawPitchRollAngles();
            AngularVelocity chVelocity =
                    controlHubIMU.getRobotAngularVelocity(AngleUnit.DEGREES);

            /* -------- Pinpoint Readings -------- */
            YawPitchRollAngles ppAngles =
                    pinpointIMU.getRobotYawPitchRollAngles();
            AngularVelocity ppVelocity =
                    pinpointIMU.getRobotAngularVelocity(AngleUnit.DEGREES);

            telemetry.addLine("====== CONTROL HUB IMU ======");
            telemetry.addData("Yaw", "%.2f", chAngles.getYaw(AngleUnit.DEGREES));
            telemetry.addData("Pitch", "%.2f", chAngles.getPitch(AngleUnit.DEGREES));
            telemetry.addData("Roll", "%.2f", chAngles.getRoll(AngleUnit.DEGREES));
            telemetry.addData("Yaw Vel", "%.2f deg/s", chVelocity.zRotationRate);

            telemetry.addLine("");
            telemetry.addLine("====== PINPOINT IMU ======");
            telemetry.addData("Yaw", "%.2f", ppAngles.getYaw(AngleUnit.DEGREES));
            telemetry.addData("Pitch", "%.2f", ppAngles.getPitch(AngleUnit.DEGREES));
            telemetry.addData("Roll", "%.2f", ppAngles.getRoll(AngleUnit.DEGREES));
            telemetry.addData("Yaw Vel", "%.2f deg/s", ppVelocity.zRotationRate);

            telemetry.update();
        }
    }
}
