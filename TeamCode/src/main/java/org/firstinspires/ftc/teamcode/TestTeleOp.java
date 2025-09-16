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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.sfdev.assembly.state.StateMachine;

@TeleOp(name = "TestTeleop")
public class TestTeleOp extends OpMode {

    // Motors for mecanum drive
    private DcMotor frontLeft, frontRight, backLeft, backRight, slidesMotor;
    boolean clawPos = false;
    boolean slidesExtend = false;  // Toggle flag for slide movement
    // Arm controller and state machine
    private ArmController armController;
    private StateMachine armStateMachine;
    boolean previousYState = false;
    boolean previousLeftBumperState = false;
    double clawRotationState = 0;
    public double pivotPos = 1;
    double clawOpen = 0.4;
    private Servo clawServo;

    private Servo pivotServo;
    public double slidesPos;

    public boolean pivotMove = false;


    @Override
    public void init() {
        // Initialize mecanum drive motors
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRight = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeft = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRight = hardwareMap.get(DcMotor.class, "backRightMotor");
        slidesMotor = hardwareMap.get(DcMotorEx.class, "slidesMotor");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        slidesMotor = hardwareMap.get(DcMotor.class, "slidesMotor");
        pivotServo = hardwareMap.get(Servo.class, "pivotServo");
        // Set motor directions for mecanum drive
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize the arm controller and state machine
        armController = new ArmController(hardwareMap, false);
        armStateMachine = armController.armMachine(gamepad1, gamepad2);  // Pass gamepad1 and gamepad2 to arm state machine
        armStateMachine.start();  // Start the state machine
        armController.armInit();
    }

    @Override


    public void loop() {
        // Mecanum drive control
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

        frontLeft.setPower(/*frontLeftPower*/0.0);
        backLeft.setPower(/*backLeftPower*/0.0);
        frontRight.setPower(frontRightPower);
        backRight.setPower(/*backRightPower*/0.0);

        // Update the arm state machine
        armStateMachine.update();
        // Update the arm controller (PID and state machine updates)
        armController.update();
        if (slidesPos == 0) slidesMotor.setPower(0);
        // Telemetry for debugging and monitoring
        telemetry.addData("Arm Position", armController.getArmPosition());
        telemetry.addData("Arm State", armStateMachine.getStateString());  // Get the current state from the state machine
        telemetry.addData("Front Left Power", frontLeft.getPower());
        telemetry.addData("Front Right Power", frontRight.getPower());
        telemetry.addData("Back Left Power", backLeft.getPower());
        telemetry.addData("Back Right Power", backRight.getPower());
        telemetry.addData("left stick y pos", gamepad1.left_stick_y);
        telemetry.addData("slides pos", slidesMotor.getCurrentPosition());
        telemetry.update();  // Update telemetry

        // Control claw position with gamepad2.y button

        if (gamepad2.dpad_up) {
            if(armController.currentState == ArmController.ArmState.SUPER_SPEC) slidesPos = -600;
            else slidesPos = -1550;
            armController.setSlidesPos(slidesPos);
        }
        if (gamepad2.dpad_down) {
            slidesPos = 0;
            armController.setSlidesPos(slidesPos);
        }

        if (gamepad2.left_bumper && !previousLeftBumperState) {
            if (clawRotationState == 0) {
                clawRotationState = 0.3;
            } else {
                clawRotationState = 0;
            }
            armController.setClawRotate(clawRotationState);
        } // Reset claw rotation
        previousLeftBumperState = gamepad2.left_bumper;

        if (slidesMotor.getCurrentPosition() > -100) {
            if (gamepad2.dpad_left) {
                slidesMotor.setPower(1);  // Extend
            } else if (gamepad2.dpad_right) {
                slidesMotor.setPower(-1); // Retract slides
                slidesExtend = false;
            } else {
                slidesMotor.setPower(0);  // Stop slides if button is not pressed
            }
        }
        else if (slidesMotor.getCurrentPosition() < -100) {
            if (gamepad2.dpad_left) {
                slidesMotor.setPower(1);  // Extend
            } else {
                slidesMotor.setPower(0);  // Stop slides if button is not pressed
            }
        }



    }
}

