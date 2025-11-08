package org.firstinspires.ftc.teamcode.opMode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.control.DriveControl;
import org.firstinspires.ftc.teamcode.control.IntakeControl;
import org.firstinspires.ftc.teamcode.control.OuttakeControl;
import org.firstinspires.ftc.teamcode.subsystem.Robot;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name = "NewTele", group = "")
public class NewTele extends LinearOpMode {

    Robot robot;
    DriveControl dc;
    IntakeControl ic;
    OuttakeControl oc;

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private double kP = 0.03;  // Proportional constant for alignment

    private final ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Robot(hardwareMap, telemetry);
        dc = new DriveControl(robot, gamepad1, gamepad2);
        ic = new IntakeControl(robot, gamepad1, gamepad2);
        oc = new OuttakeControl(robot, gamepad1, gamepad2);

        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        waitForStart();
        if (isStopRequested()) return;

        robot.init();

        while (opModeIsActive()) {
            robot.update();
            dc.update();
            ic.update();
            oc.update();

            // TODO: add state machine either here or in robot class
        }


    }
}
