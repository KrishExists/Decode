/* package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.sfdev.assembly.state.StateMachine;

@TeleOp

public class MecanumTeleOp extends LinearOpMode {
    //private ArmController armController;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        DcMotor slidesMotor = null;
        DcMotor armMotorLeft = null;
        DcMotor armMotorRight = null;

        // Initialize motors
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        slidesMotor = hardwareMap.get(DcMotor.class, "slidesMotor");
        armMotorLeft = hardwareMap.get(DcMotor.class, "armMotorLeft");
        armMotorRight = hardwareMap.get(DcMotor.class, "armMotorRight");

        double motorPower = 0.0;
        double armMotorPowerLeft = 0.0;
        double armMotorPowerRight = 0.0;
        frontLeftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        //armController = new ArmController(hardwareMap, "armMotor", "clawRotate", "spinTake");

        // Reverse the right side motors
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        slidesMotor.setDirection(DcMotor.Direction.FORWARD);
        armMotorLeft.setDirection(DcMotorEx.Direction.REVERSE);
        armMotorRight.setDirection(DcMotorSimple.Direction.FORWARD);

        slidesMotor.setPower(0);
        armMotorLeft.setPower(0);
        armMotorRight.setPower(0);

        ArmController armController = null;
        StateMachine armMachine = armController.armMachine(gamepad1, gamepad2);

        waitForStart();
        armController.armInit();
        armMachine.setState(ArmController.ArmState.NEUTRAL_POSITION);
        armMachine.start();

        while (opModeIsActive()) {
            armMachine.update();
            telemetry.addData("State", armMachine.getStateString());
            telemetry.update();

            // Drive control
            double y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;
            double b = gamepad1.left_trigger;

            // Calculate motor power
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;
            /*if (gamepad1.a) {
                motorPower = 1.0;
            }
            else if (gamepad1.b) {
                motorPower = -1.0;
            }

            else if (gamepad1.dpad_down) {
                armMotorPowerLeft = 0.5;
                armMotorPowerRight = 0.5;
            }

            else if (gamepad1.dpad_up) {
                armMotorPowerLeft = -0.5;
                armMotorPowerRight = -0.5;
            }
            else {
                motorPower = 0.0;
                armMotorPowerRight = 0.0;
                armMotorPowerLeft = 0.0;
            }

            slidesMotor.setPower(motorPower);
            armMotorLeft.setPower(armMotorPowerLeft);
            armMotorRight.setPower(armMotorPowerRight);

            // Set motor power
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            // Control end effector speed
            /*double endEffectorSpeed = 0;
            if (gamepad2.right_trigger > 0.1) {
                endEffectorSpeed = 1; // Forward
            } else if (gamepad2.left_bumper) {
                endEffectorSpeed = -1; // Reverse
            }
            armController.setEndEffectorSpeed(endEffectorSpeed);
        }


    }
}
*/


/*
@TeleOp
public class MecanumTeleOp extends OpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private ArmController armController;
    private StateMachine armStateMachine;
    private HardwareMap hardwareMap;
    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRight = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeft = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRight = hardwareMap.get(DcMotor.class, "backRightMotor");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        armController = new ArmController(hardwareMap);
        armStateMachine = armController.armMachine(gamepad1, gamepad2);
        armStateMachine.start();
    }

    @Override
    public void loop() {
        double drive = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double rotate = -gamepad1.right_stick_x;

        frontLeft.setPower(drive + strafe + rotate);
        backLeft.setPower(drive - strafe + rotate);
        frontRight.setPower(drive - strafe - rotate);
        backRight.setPower(drive + strafe - rotate);

        armStateMachine.update();

        armController.update();

        telemetry.addData("Arm Position", armController.getArmPosition());
        telemetry.addData("Arm State", armStateMachine.getStateString());
        telemetry.addData("Front Left Power", frontLeft.getPower());
        telemetry.addData("Front Right Power", frontRight.getPower());
        telemetry.addData("Back Left Power", backLeft.getPower());
        telemetry.addData("Back Right Power", backRight.getPower());
        telemetry.update();
    }

    @Override
    public void stop() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        armController.stopArm();
    }
}
*/
/*
public class MecanumTeleOp extends OpMode {

    // Motors for mecanum drive
    private DcMotor frontLeft, frontRight, backLeft, backRight, slidesMotor;
    boolean clawPos = false;
    boolean slidesExtend = false;
    // Arm controller and state machine
    private ArmController armController;
    private StateMachine armStateMachine;
    @Override
    public void init() {
        // Initialize mecanum drive motors
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRight = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeft = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRight = hardwareMap.get(DcMotor.class, "backRightMotor");
        slidesMotor = hardwareMap.get(DcMotorEx.class, "slidesMotor");

        // Set motor directions for mecanum drive
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        // Initialize the arm controller and state machine
        armController = new ArmController(hardwareMap);
        armStateMachine = armController.armMachine(gamepad1, gamepad2);  // Pass gamepad1 and gamepad2 to arm state machine
        armStateMachine.start();  // Start the state machine
        armController.armInit();
    }

    @Override
    public void loop() {
        // Mecanum drive control
        double drive = -gamepad1.left_stick_y;  // Forward/backward movement
        double strafe = -gamepad1.left_stick_x; // Left/right movement
        double rotate = -gamepad1.right_stick_x; // Rotation (turning)

        slidesMotor.setPower(gamepad2.left_stick_y);

        // Set the power for the mecanum drive motors
        frontLeft.setPower(drive + strafe + rotate);
        backLeft.setPower(drive - strafe + rotate);
        frontRight.setPower(drive - strafe - rotate);
        backRight.setPower(drive + strafe - rotate);

        // Update the arm state machine
        armStateMachine.update();

        // Update the arm controller (PID and state machine updates)
        armController.update();

        // Telemetry for debugging and monitoring
        telemetry.addData("Arm Position", armController.getArmPosition());
        telemetry.addData("Arm State", armStateMachine.getStateString());  // Get the current state from the state machine
        telemetry.addData("Front Left Power", frontLeft.getPower());
        telemetry.addData("Front Right Power", frontRight.getPower());
        telemetry.addData("Back Left Power", backLeft.getPower());
        telemetry.addData("Back Right Power", backRight.getPower());
        telemetry.update();  // Update telemetry
        if(gamepad2.y && !clawPos){
            clawPos = true;
            armController.clawControl(0);
        }
        else if(gamepad2.y && clawPos){
            clawPos = false;
            armController.clawControl(1);
        }
        if(gamepad2.right_bumper) armController.setClawRotate(1);
        if(gamepad2.left_bumper) armController.setClawRotate(0);

//        if(gamepad2.dpad_up && !slidesExtend) {
//            armController.extend(1550);
//            slidesExtend = true;
//        }
//        if(gamepad2.dpad_down && slidesExtend) {
//            armController.extend(0);
//            slidesExtend = false;
//        }
    }

    @Override
    public void stop() {
        // Stop all motors when teleop is stopped
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        // Stop the arm control system
        armController.stopArm();
    }
}
*/


package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.arcrobotics.ftclib.util.InterpLUT;

import java.util.List;


@TeleOp(name = "MecanumTeleop", group = "Concept")
public class MecanumTeleOp extends OpMode {
    decodeArm arm;
    double y;
    double x;
    double rx;
    boolean driveForward;
    // Motors for mecanum drive
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    boolean clawPos = false;
    boolean slidesExtend = false;  // Toggle flag for slide movement
    // Arm controller and state machine
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;
    boolean previousYState = false;
    boolean previousLeftBumperState = false;
    double clawRotationState = 0;
    public double pivotPos = 1;
    double clawOpen = 0.4;
    private Servo clawServo;

    private Servo pivotServo;
    public double slidesPos;
    public String motif;

    public boolean pivotMove = false;
// 0 backLeftMotor, 1 backRightMotor, 2 frontRightMotor, 3 frontLeftMotor

    @Override
    public void init() {
        driveForward = true;
        // Initialize mecanum drive motors
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRight = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeft = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRight = hardwareMap.get(DcMotor.class, "backRightMotor");
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        initAprilTag();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();
        // Initialize the arm controller and state machine

    }

    @Override


    public void loop() {
        telemetryAprilTag();
        // Save CPU resources; can resume streaming when needed.
        if (gamepad1.dpad_down) {
            visionPortal.stopStreaming();
        } else if (gamepad1.dpad_up) {
            visionPortal.resumeStreaming();
        }

        // Mecanum drive control

        if(driveForward) {
            y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            rx = gamepad1.right_stick_x;
        }else{
            x = -gamepad1.left_stick_y * 1.1;
            y = gamepad1.left_stick_x * 1.1;
            rx = -gamepad1.right_stick_x;
        }
        if(gamepad1.dpad_down){
            driveForward = !driveForward;
        }
        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);

        // Update the arm state machine
        // Telemetry for debugging and monitoring
        telemetry.addData("Front Left Power", frontLeft.getPower());
        telemetry.addData("Front Right Power", frontRight.getPower());
        telemetry.addData("Back Left Power", backLeft.getPower());
        telemetry.addData("Back Right Power", backRight.getPower());
        telemetry.addData("left stick y pos", gamepad1.left_stick_y);
        telemetry.addData("Drive forward is",driveForward);
        telemetry.update();  // Update telemetry

        // Control claw position with gamepad2.y button







    }
    private void initAprilTag() {

        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, aprilTag);
        }

    }

    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
            if(detection.id==21){
                motif = "gpp";
            }
            if(detection.id==22){
                motif = "pgp";
            } if(detection.id==23){
                motif = "ppg";
            }

        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine(String.format("Motif is %s",motif));
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }   // end method telemetryAprilTag()
}

