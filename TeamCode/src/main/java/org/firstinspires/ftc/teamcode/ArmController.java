/*package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;
@Config
public class ArmController {

    private DcMotorEx armMotor;
    private Servo pivotServo;
    private CRServo endEffectorServo;
    public enum ArmState{
        ground,
        low,
        HighBar,
        lowBask, NEUTRAL, INTAKE,
        hang
    }
    static final double COUNTS_PER_REV = 1425.1;
    static final double GEAR_RATIO = 1.0;
    static final double COUNTS_PER_DEGREE = (COUNTS_PER_REV * GEAR_RATIO) / 360.0;
    public static double lowBasket = 105;
    public static double lowBar = 90;
    public static double highBar = 120;
    public static double lowAscent = 100;
    public static double startPos = 30;
    public static double specimenPivot = 0.7;
    boolean lowBasketGamepad = false;
    boolean highBarGamepad = false;
    boolean hangGamepad = false;
    public static double hang = 60;
    public ArmController(HardwareMap hardwareMap, String motorName, String pivotServoName, String endEffectorServoName) {
        armMotor = hardwareMap.get(DcMotorEx.class, motorName);
        pivotServo = hardwareMap.get(Servo.class, pivotServoName);
        endEffectorServo = hardwareMap.get(CRServo.class, endEffectorServoName);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        pivotServo.setPosition(0.4);
    }

    public StateMachine armMachine(Gamepad gamepad1, Gamepad gamepad2) {
        return new StateMachineBuilder()
                .state(ArmState.NEUTRAL)
                .loop(() -> {
                    moveArmToPosition(startPos,0.1);
                    if(gamepad2.a ) lowBasketGamepad = true;
                })
                .transition(() -> gamepad2.right_trigger>0, ArmState.INTAKE)
                .transition(() -> !gamepad2.a && lowBasketGamepad, ArmState.lowBask, () -> lowBasketGamepad = false)
                .transition(() -> gamepad2.y, ArmState.HighBar, () -> {
                    highBarGamepad = true;
                })
                .transition(() -> gamepad2.x, ArmState.hang, ()-> {
                    hangGamepad = true;
                })


                .state(ArmState.INTAKE)
                .loop(()-> {
                    moveArmToPosition(0,0.3);
                    setEndEffectorSpeed(1);
                })
                .transition(() -> gamepad2.right_trigger == 0, ArmState.NEUTRAL)

                .state(ArmState.lowBask)
                .loop(() -> {
                    moveArmToPosition(lowBasket, 0.1);
                    if(gamepad2.a ) lowBasketGamepad = true;

                })
                .transition(() -> !gamepad2.a && lowBasketGamepad, ArmState.NEUTRAL, () -> lowBasketGamepad = false)
                .state(ArmState.HighBar)
                .loop(() ->{
                    moveArmToPosition(highBar, 0.1);
                    if(!gamepad2.y) highBarGamepad = false;
                    pivotServo.setPosition(1);
                })
                .transition(() -> gamepad2.y && !highBarGamepad, ArmState.NEUTRAL)
                .state(ArmState.hang)
                .loop(() ->{
                    moveArmToPosition(hang, 0.1);
                    if(!gamepad2.x) hangGamepad = false;
                })
                .transition(() -> gamepad2.x && !hangGamepad, ArmState.NEUTRAL)
                .build();
    }
    public int degreesToCounts(double degrees) {
        return (int)(degrees * COUNTS_PER_DEGREE);
    }
    public void moveArmToPosition(double degrees, double power) {
        int targetPosition = degreesToCounts(degrees);
        armMotor.setTargetPosition(targetPosition);
        armMotor.setPower(power);
    }

    // Convert an angle (0 to 180 degrees) to a servo position (0.0 to 1.0)
    // For continuous rotation: Set speed for the end-effector servo (0.0 to 1.0, with 0.5 as stop)
    public void setEndEffectorSpeed(double speed) {
        endEffectorServo.setPower(speed);
    }
    public boolean isBusy() {
        return armMotor.isBusy();
    }

    public void stopArm() {
        armMotor.setPower(0);
    }

    public int getCurrentPosition() {
        return armMotor.getCurrentPosition();
    }
} This is the old code. If new code doesn't work return to this code

 */
/*
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

@Config
public class ArmController {

    private DcMotorEx armMotor;
    private Servo pivotServo;
    private CRServo endEffectorServo;
    private ArmState currentState;

    public static double NEW_DOWN_POSITION = 180;
    public static double UP_POSITION = 90;
    public static double INTAKE_SPEED = 1.0;
    public static double OUTTAKE_SPEED = -1.0;
    PIDFCoefficients armCoefficients;

    public ArmController(HardwareMap hardwareMap, String motorName, String pivotServoName, String endEffectorServoName) {
        armMotor = hardwareMap.get(DcMotorEx.class, motorName);
        pivotServo = hardwareMap.get(Servo.class, pivotServoName);
        endEffectorServo = hardwareMap.get(CRServo.class, endEffectorServoName);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }
    public void armInit(){
        pivotServo.setPosition(0.5);
        currentState = ArmState.NEUTRAL;
    }

    public enum ArmState {
        NEUTRAL,
        MOVE_UP,
        STAY_UP,
        MOVE_DOWN,
        STAY_DOWN,
        INTAKE,
        OUTTAKE
    }

    public StateMachine armMachine(Gamepad gamepad1, Gamepad gamepad2) {
        return new StateMachineBuilder()
                .state(ArmState.NEUTRAL)
                .loop(() -> {
                    moveArmToPosition(NEW_DOWN_POSITION, 0.3);
                    currentState = ArmState.NEUTRAL;
                })
                .transition(() -> gamepad2.a && currentState != ArmState.MOVE_UP, ArmState.MOVE_UP)
                .transition(() -> gamepad2.b && currentState != ArmState.MOVE_DOWN, ArmState.MOVE_DOWN)

                .state(ArmState.MOVE_UP)
                .loop(() -> {
                    moveArmToPosition(UP_POSITION, 0.3);
                    currentState = ArmState.MOVE_UP;
                })
                .transition(() -> !gamepad2.a, ArmState.STAY_UP)
                .transition(() -> gamepad2.b && currentState != ArmState.MOVE_DOWN, ArmState.MOVE_DOWN)

                .state(ArmState.STAY_UP)
                .loop(() -> {
                    moveArmToPosition(UP_POSITION, 0.3);
                    currentState = ArmState.STAY_UP;
                })
                .transition(() -> gamepad2.b && currentState != ArmState.MOVE_DOWN, ArmState.MOVE_DOWN)

                .state(ArmState.MOVE_DOWN)
                .loop(() -> {
                    moveArmToPosition(NEW_DOWN_POSITION, 0.3);
                    currentState = ArmState.MOVE_DOWN;
                })
                .transition(() -> !gamepad2.b, ArmState.STAY_DOWN)
                .transition(() -> gamepad2.a && currentState != ArmState.MOVE_UP, ArmState.MOVE_UP)

                .state(ArmState.STAY_DOWN)
                .loop(() -> {
                    moveArmToPosition(NEW_DOWN_POSITION, 0.3);
                    currentState = ArmState.STAY_DOWN;
                })
                .transition(() -> gamepad2.a && currentState != ArmState.MOVE_UP, ArmState.MOVE_UP)

                .transition(() -> gamepad2.right_trigger > 0, ArmState.INTAKE)
                .transition(() -> gamepad2.left_trigger > 0, ArmState.OUTTAKE)

                .state(ArmState.INTAKE)
                .loop(() -> {
                    setEndEffectorSpeed(INTAKE_SPEED);
                    currentState = ArmState.INTAKE;
                })
                .transition(() -> gamepad2.right_trigger == 0, ArmState.NEUTRAL)

                .state(ArmState.OUTTAKE)
                .loop(() -> {
                    setEndEffectorSpeed(OUTTAKE_SPEED);
                    currentState = ArmState.OUTTAKE;
                })
                .transition(() -> gamepad2.left_trigger == 0, ArmState.NEUTRAL)
                .build();
    }

    public void moveArmToPosition(double degrees, double power) {
        int targetPosition = degreesToCounts(degrees);
        armMotor.setTargetPosition(targetPosition);
        armMotor.setPower(power);
    }
    public int getArmPosition(){
        return armMotor.getCurrentPosition();
    }
    public int degreesToCounts(double degrees) {
        final double COUNTS_PER_REV = 1425.1; // Adjust as necessary for your motor
        final double GEAR_RATIO = 1.0; // Adjust if necessary
        final double COUNTS_PER_DEGREE = (COUNTS_PER_REV * GEAR_RATIO) / 360.0;
        return (int)(degrees * COUNTS_PER_DEGREE);
    }

    public void setEndEffectorSpeed(double speed) {
        endEffectorServo.setPower(speed);
    }

    public void stopArm() {
        armMotor.setPower(0);
    }
}

 */

/*package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

@Config
public class ArmController {

    private DcMotorEx armMotorLeft;
    private DcMotorEx armMotorRight;
    private ArmState currentState;
    public static int INTAKE_POSITION = 970;//change that based on position in telemtery
    public static int High_BASKET = 1550;
    public static int NEUTRAL_POSITION = 457;
    public static int STAGING_POSITION = 807;
    public static double START_POSITION = 0;
    public static double INTAKE_SPEED = 1.0;
    public static double OUTTAKE_SPEED = -1.0;
    boolean stagingToggle = false;
    boolean lowBasketToggle = false;
    boolean intakeToggle = false;
    public PIDController pidController;

    public ArmController(HardwareMap hardwareMap) {
        armMotorLeft = hardwareMap.get(DcMotorEx.class, "armMotorLeft");
        armMotorRight = hardwareMap.get(DcMotorEx.class, "armMotorRight");
        armMotorLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        armMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotorLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotorRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        armMotorLeft.setDirection(DcMotorEx.Direction.REVERSE);
        armMotorRight.setDirection(DcMotorSimple.Direction.FORWARD);
        pidController = new PIDController(PIDF_Arm.kp, PIDF_Arm.ki, PIDF_Arm.kd, PIDF_Arm.kcos);
    }
    public void armInit(){
        currentState = ArmState.NEUTRAL_POSITION;
    }

    public enum ArmState {
        NEUTRAL_POSITION,
        STAGING,
        INTAKE,
        LOW_BASKET,
    }
    public StateMachine armMachine(Gamepad gamepad1, Gamepad gamepad2) {
        return new StateMachineBuilder()
                .state(ArmState.NEUTRAL_POSITION)
                .loop(() -> {
                    setArmPosition(NEUTRAL_POSITION);
                    if(gamepad2.a) stagingToggle = true;
                    if(gamepad2.x) lowBasketToggle = true;
                })
                .transition(() -> !gamepad2.a && stagingToggle, ArmState.STAGING)
                .transition(() -> !gamepad2.x && lowBasketToggle, ArmState.LOW_BASKET)
                .state(ArmState.STAGING)
                .loop(() -> {
                    setArmPosition(STAGING_POSITION);
                    if(gamepad2.a) stagingToggle = false;
                    if(gamepad2.b) intakeToggle = true;
                })
                .transition(() -> !gamepad2.a && !stagingToggle, ArmState.NEUTRAL_POSITION)
                .transition(() -> !gamepad2.b && intakeToggle, ArmState.INTAKE)
                .state(ArmState.INTAKE)
                .loop(() -> {
                    setArmPosition(INTAKE_POSITION);
                    if(gamepad2.b) intakeToggle = false;
                }) // Maintain position with PID
                .transition(() -> !gamepad2.b && !intakeToggle, ArmState.STAGING)
                .state(ArmState.LOW_BASKET)
                .loop(() -> {
                    setArmPosition(LOW_BASKET);
                    if(gamepad2.x) lowBasketToggle = false;
                })
                .transition(() -> !gamepad2.x && !lowBasketToggle, ArmState.NEUTRAL_POSITION)
                .build();
    }

    private void setArmPosition(double targetPosition) {
        int currentPosition = armMotorLeft.getCurrentPosition();
        double output = pidController.calculate(targetPosition, currentPosition);

        // Set motor power based on PID output
        armMotorLeft.setPower(output);
        armMotorRight.setPower(output);
    }


    public int degreesToCounts(double degrees) {
        final double COUNTS_PER_REV = 1425.1;
        final double GEAR_RATIO = 1.0;
        final double COUNTS_PER_DEGREE = (COUNTS_PER_REV * GEAR_RATIO) / 360.0;
        return (int)(degrees * COUNTS_PER_DEGREE);
    }


    public void stopArm() {
        armMotorLeft.setPower(0);
        armMotorRight.setPower(0);
        pidController.reset(); // Reset PID controller
    }
    public int getArmPosition(){
        return armMotorLeft.getCurrentPosition();
    }
}

// PID Controller Class
*/

package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

@Config
public class ArmController{

    // Motors
    private DcMotorEx armControllerLeft;
    private DcMotorEx armControllerRight;

    public ArmState currentState;
    // Servos
    private Servo wristLeft;
    private Servo wristRight;
    private Servo pivotServo;
    private Servo clawServo;
    private Servo clawRotate;
    private boolean hi = false;

    // PID Controller
    private PIDController pidController, slidesController;

    // Positions (in encoder counts or degrees)
    public static int HIGH_BASKET_POSITION = 600;//851
    public static int LOW_BASKET_POSITION = 800;//851
    public static int LOW_CHAMBER_POSITION = 300;//851
    public static int HIGH_CHAMBER_POSITION = 1200;//851
    public static int INTAKE_POSITION = 350;
    public static int TARGET_POSITION = 0;
    public static int slidesTarget = 0;
    double slidesPower = 0;

    public DcMotor slidesMotor;
    public static int STAGE_POSITION = -40;
    public static int HOVER_POSITION = 60;
    public static double START_POSITION = 200;
    public static double INTAKE_SPEED = 1.0;
    public static double OUTTAKE_SPEED = -1.0;

    // Toggles for state transitions
    boolean highBasketToggle = false;
    boolean intakeStaging = false;
    boolean outtakeToggle = false;
    boolean hoverToggle = false;

    boolean clawOpen = false;
    boolean clawOpen2 = false;
    boolean clawOpen3 = false;
    public double clawPos = 0;
    public double pivotPos = 1;
    public boolean pivotMove = false;
    public double swivelPos = 0;

    public ArmController(HardwareMap hardwareMap, boolean auto) {
        // Initialize motors
        armControllerLeft = hardwareMap.get(DcMotorEx.class, "armMotorLeft");
        armControllerRight = hardwareMap.get(DcMotorEx.class, "armMotorRight");
        slidesMotor = hardwareMap.get(DcMotor.class, "slidesMotor");

        // Set motor behaviors
        armControllerLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        armControllerRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        armControllerLeft.setDirection(DcMotorEx.Direction.FORWARD); // Reversed
        armControllerRight.setDirection(DcMotorSimple.Direction.REVERSE); // Not reversed
        if (auto) {
            armControllerLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            armControllerRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        }
        slidesMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armControllerLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        armControllerRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        slidesMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize servos
        wristLeft = hardwareMap.get(Servo.class, "wristLeft");
        wristRight = hardwareMap.get(Servo.class, "wristRight");
        pivotServo = hardwareMap.get(Servo.class, "pivotServo");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        clawRotate = hardwareMap.get(Servo.class, "clawRotate");

        // Reversed wrist servos
        wristLeft.setDirection(Servo.Direction.REVERSE);
        wristRight.setDirection(Servo.Direction.FORWARD);

        // PID controller initialization
        pidController = new PIDController(PIDF_Arm.kp, PIDF_Arm.ki, PIDF_Arm.kd, PIDF_Arm.kcos);
        slidesController = new PIDController(slidesPID.kp, slidesPID.ki, slidesPID.kd, 0);
    }
    public void armInit() {
        // Set initial arm position to neutral or starting position
        currentState = ArmState.INTAKE_POSITION;
    }

    public enum ArmState {
        HIGH_BASKET,
        INTAKE_POSITION,
        INTAKE_STAGE,
        INTAKE_STAGE2,
        OUTTAKE_STAGE,
        OUTTAKE_STAGE2,
        HOVER_STAGE,
        PRE_LOAD,
        SPEC_Stage,
        AUTO_HIGH_BASKET_1,
        AUTO_HIGH_BASKET_2,
        AUTO_PICK_UP_1,
        AUTO_PICK_UP_2,
        SPECIMEN1,
        SPECIMEN2,
        SPECIMEN3,
        AUTO_OUTTAKE1,
        AUTO_OUTTAKE2,
        SPECIMEN_PICKUP1,
        SPECIMEN_PICKUP2,
        SUPER_SPEC,
        AUTO_SPECIMEN2,
        AUTO_SPECIMEN3,
        AUTO_SUPER_SPEC,
        AUTO_SPECIMEN_PICKUP1
    }

    // Add wasBButtonPressed as a class-level member variable
    private boolean wasBButtonPressed = false;
    private boolean wasXButtonPressed = false;
    private boolean wasYButtonPressed = false;
    private boolean wasAButtonPressed = false;

    public StateMachine armMachine(Gamepad gamepad1, Gamepad gamepad2) {
        return new StateMachineBuilder()
                .state(ArmState.PRE_LOAD)
                .onEnter(() -> {
                    currentState = ArmState.PRE_LOAD;
                    clawServo.setPosition(0);
                })
                .transitionTimed(0.2, ArmState.INTAKE_POSITION)
                // **INTAKE_POSITION** State: This is the starting state
                .state(ArmState.INTAKE_POSITION)
                .loop(() -> {
                    // Set arm to intake position
                    currentState = ArmState.INTAKE_POSITION;
                    setArmPosition(INTAKE_POSITION);
                    clawServo.setPosition(0);
                    wristLeft.setPosition(0.77);
                    pivotServo.setPosition(1); // Example pivot position
                    clawRotate.setPosition(0);
                })
                // Transition to HOVER_STAGE when falling edge of b is detected
                .transition(() -> {
                    boolean isBButtonPressed = gamepad2.b;
                    boolean fallingEdgeDetected = !wasBButtonPressed && isBButtonPressed;
                    wasBButtonPressed = isBButtonPressed;
                    return fallingEdgeDetected;
                }, ArmState.HOVER_STAGE)
                .transition(() -> {
                    boolean isAButtonPressed = gamepad2.a;
                    boolean fallingEdgeDetected = !wasAButtonPressed && isAButtonPressed;
                    wasAButtonPressed = isAButtonPressed;
                    return fallingEdgeDetected;
                }, ArmState.SPECIMEN_PICKUP1)

                // **HOVER_STAGE** State
                .state(ArmState.HOVER_STAGE)
                .loop(() -> {
                    // Set arm to hover position
                    currentState = ArmState.HOVER_STAGE;
                    clawServo.setPosition(0.4);
                    wristLeft.setPosition(1);
                    setArmPosition(150);
                    pivotServo.setPosition(1);
                })
                // Transition to INTAKE_STAGE when falling edge of b is detected (back from hover)
                .transition(() -> {
                    boolean isBButtonPressed = gamepad2.b;
                    boolean fallingEdgeDetected = !wasBButtonPressed && isBButtonPressed;
                    wasBButtonPressed = isBButtonPressed;
                    return fallingEdgeDetected;
                }, ArmState.INTAKE_STAGE)
                .transition(() -> {
                    boolean isAButtonPressed = gamepad2.a;
                    boolean fallingEdgeDetected = !wasAButtonPressed && isAButtonPressed;
                    wasAButtonPressed = isAButtonPressed;
                    return fallingEdgeDetected;
                }, ArmState.SPECIMEN_PICKUP1)

                // **INTAKE_STAGE** State
                .state(ArmState.INTAKE_STAGE)
                .loop(() -> {
                    // Set arm to intake staging position
                    currentState = ArmState.INTAKE_STAGE;
                    wristLeft.setPosition(1);
                    clawServo.setPosition(0.4);
                    pivotServo.setPosition(1);
                    setArmPosition(-10);
                })
                .transitionTimed(0.2, ArmState.INTAKE_STAGE2)
                .state(ArmState.INTAKE_STAGE2)
                .loop(() -> {
                    currentState = ArmState.INTAKE_STAGE2;
                    setArmPosition(-10);
                    wristLeft.setPosition(1);
                    if(gamepad2.right_trigger > 0.1 && !pivotMove){
                        if(pivotPos==1){
                            pivotPos = 0;
                            swivelPos = 0.67;
                        }
                        else {
                            pivotPos = 1;
                            swivelPos = 0;
                        }
                        pivotServo.setPosition(pivotPos);
                        clawRotate.setPosition(swivelPos);
                    }
                    pivotMove = gamepad2.right_trigger > 0.1;
                    clawServo.setPosition(0);
                })
                // Transition to HOVER_STAGE when falling edge of b is detected
                .transition(() -> {
                    boolean isBButtonPressed = gamepad2.b;
                    boolean fallingEdgeDetected = !wasBButtonPressed && isBButtonPressed;
                    wasBButtonPressed = isBButtonPressed;
                    return fallingEdgeDetected;
                }, ArmState.HOVER_STAGE)
                // Transition to HIGH_BASKET when X is pressed (progress to next state)
                .transition(() -> {
                    boolean isXButtonPressed = gamepad2.x;
                    boolean fallingEdgeDetected = !wasXButtonPressed && isXButtonPressed;
                    wasXButtonPressed = isXButtonPressed;
                    return fallingEdgeDetected;
                }, ArmState.HIGH_BASKET)
                .transition(() -> {
                    boolean isYButtonPressed = gamepad2.y;
                    boolean fallingEdgeDetected = !wasYButtonPressed && isYButtonPressed;
                    wasYButtonPressed = isYButtonPressed;
                    return fallingEdgeDetected;
                }, ArmState.SPECIMEN1)
                .transition(() -> {
                    boolean isAButtonPressed = gamepad2.a;
                    boolean fallingEdgeDetected = !wasAButtonPressed && isAButtonPressed;
                    wasAButtonPressed = isAButtonPressed;
                    return fallingEdgeDetected;
                }, ArmState.SPECIMEN_PICKUP1)
                // **HIGH_BASKET** State
                .state(ArmState.HIGH_BASKET)
                .loop(() -> {
                    // Set arm to high basket position
                    currentState = ArmState.HIGH_BASKET;
                    clawServo.setPosition(clawPos);
                    setArmPosition(650);
                    wristLeft.setPosition(1);
                    clawRotate.setPosition(0);
                    pivotServo.setPosition(0.25);
                    if(gamepad2.right_bumper && !clawOpen){
                        if(clawPos == 0) clawPos = 0.4;
                        else clawPos = 0;
                        clawServo.setPosition(clawPos);
                    }
                    clawOpen = gamepad2.right_bumper;
                })
                /*

                */

                // Transition to OUTTAKE_STAGE when Y is pressed (progress to next state)
                .transition(() -> {
                    boolean isXButtonPressed = gamepad2.x;
                    boolean fallingEdgeDetected = !wasXButtonPressed && isXButtonPressed;
                    wasXButtonPressed = isXButtonPressed;
                    return fallingEdgeDetected;
                }, ArmState.OUTTAKE_STAGE)

                // **OUTTAKE_STAGE** State
                .state(ArmState.OUTTAKE_STAGE)
                .loop(() -> {
                    // Set arm to outtake position
                    currentState = ArmState.OUTTAKE_STAGE;
                    clawPos = 0;
                    setArmPosition(615);
                    pivotServo.setPosition(1);
                    wristLeft.setPosition(0.5);
                    clawRotate.setPosition(0);
                })
                .transitionTimed(0.3,ArmState.OUTTAKE_STAGE2)
                .state(ArmState.OUTTAKE_STAGE2)
                .loop(() -> {
                    clawPos = 0;
                    setArmPosition(500);
                    wristLeft.setPosition(0.5);
                    clawRotate.setPosition(0);
                    pivotServo.setPosition(1);
                    setSlidesPos(0);
                })
                // Transition back to INTAKE_POSITION to repeat the sequence indefinitely
                .transition(() -> {
                    boolean isXButtonPressed = gamepad2.x;
                    boolean fallingEdgeDetected = !wasXButtonPressed && isXButtonPressed;
                    wasXButtonPressed = isXButtonPressed;
                    return fallingEdgeDetected;
                }, ArmState.INTAKE_POSITION)
                .state(ArmState.AUTO_HIGH_BASKET_1)
                .loop(() -> {
                    currentState = ArmState.AUTO_HIGH_BASKET_1;
                    setArmPosition(640);
                    clawServo.setPosition(0);
                    wristLeft.setPosition(0.77);
                    pivotServo.setPosition(1); // Example pivot position
                    clawRotate.setPosition(0);
                })
                .state(ArmState.AUTO_HIGH_BASKET_2)
                .loop(() -> {
                    currentState = ArmState.AUTO_HIGH_BASKET_2;
                    clawServo.setPosition(clawPos);
                    setArmPosition(640);
                    wristLeft.setPosition(1);
                    clawRotate.setPosition(0);
                    pivotServo.setPosition(0.2);
                    if(gamepad2.right_bumper && !clawOpen){
                        if(clawPos == 0) clawPos = 0.4;
                        else clawPos = 0;
                        clawServo.setPosition(clawPos);
                    }
                    clawOpen = gamepad2.right_bumper;
                })
                .transition(() -> hi, ArmState.AUTO_HIGH_BASKET_1)
                .state(ArmState.AUTO_PICK_UP_1)
                .loop(() -> {
                    wristLeft.setPosition(1);
                    clawServo.setPosition(0.4);
                    pivotServo.setPosition(1);
                    setArmPosition(-20);
                })
                .state(ArmState.AUTO_PICK_UP_2)
                .loop(() -> {
                    wristLeft.setPosition(1);
                    clawServo.setPosition(0);
                    pivotServo.setPosition(1);
                    setArmPosition(-20);
                })
                .transition(() -> hi, ArmState.AUTO_PICK_UP_1)
                .state(ArmState.SPECIMEN1)
                .loop(() -> {
                    clawPos = 0;
                    setArmPosition(375);
                    clawServo.setPosition(0);
                    pivotServo.setPosition(0.4);
                    wristLeft.setPosition(1);
                })
                .transition(() -> {
                    boolean isYButtonPressed = gamepad2.y;
                    boolean fallingEdgeDetected = !wasYButtonPressed && isYButtonPressed;
                    wasYButtonPressed = isYButtonPressed;
                    return fallingEdgeDetected;
                }, ArmState.SPECIMEN2)
                .transition(() -> {
                    boolean isAButtonPressed = gamepad2.a;
                    boolean fallingEdgeDetected = !wasAButtonPressed && isAButtonPressed;
                    wasAButtonPressed = isAButtonPressed;
                    return fallingEdgeDetected;
                }, ArmState.SUPER_SPEC)
                .state(ArmState.SPECIMEN2)
                .loop(() -> {
                    setArmPosition(150);
                    clawServo.setPosition(0);
                    pivotServo.setPosition(0.4);
                    wristLeft.setPosition(1);
                })
                .transitionTimed(0.4, ArmState.SPECIMEN3)
                .state(ArmState.SPECIMEN3)
                .loop(() -> {
                    setArmPosition(150);
                    clawServo.setPosition(0.4);
                    pivotServo.setPosition(0.4);
                    wristLeft.setPosition(1);
                })
                .transition(() -> {
                    boolean isYButtonPressed = gamepad2.y;
                    boolean fallingEdgeDetected = !wasYButtonPressed && isYButtonPressed;
                    wasYButtonPressed = isYButtonPressed;
                    return fallingEdgeDetected;
                }, ArmState.INTAKE_POSITION)
                .state(ArmState.AUTO_OUTTAKE1)
                .loop(() -> {
                    clawPos = 0;
                    pivotServo.setPosition(1);
                    setArmPosition(630);
                    wristLeft.setPosition(0.5);
                    clawRotate.setPosition(0);
                })
                .state(ArmState.AUTO_OUTTAKE2)
                .loop(() -> {
                    clawPos = 0;
                    setArmPosition(500);
                    wristLeft.setPosition(0.5);
                    clawRotate.setPosition(0);
                    pivotServo.setPosition(1);
                    setSlidesPos(0);
                })
                .transition(() -> hi, ArmState.AUTO_OUTTAKE1)
                .state(ArmState.SPECIMEN_PICKUP1)
                .loop(() -> {
                    setArmPosition(80);
                    wristLeft.setPosition(1);
                    pivotServo.setPosition(0.5);
                    clawServo.setPosition(clawPos);
                    if(gamepad2.right_bumper && !clawOpen2){
                        if(clawPos == 0) clawPos = 0.5;
                        else clawPos = 0;
                        clawServo.setPosition(clawPos);
                    }
                    clawOpen2 = gamepad2.right_bumper;
                })
                .transition(() -> {
                    boolean isAButtonPressed = gamepad2.a;
                    boolean fallingEdgeDetected = !wasAButtonPressed && isAButtonPressed;
                    wasAButtonPressed = isAButtonPressed;
                    return fallingEdgeDetected;
                }, ArmState.SPECIMEN1)
                .state(ArmState.SUPER_SPEC)
                .loop(() -> {
                    currentState = ArmState.SUPER_SPEC;
                    clawServo.setPosition(clawPos);
                    setArmPosition(690);
                    wristLeft.setPosition(1);
                    pivotServo.setPosition(0.2);
                    if(gamepad2.right_bumper && !clawOpen3){
                        if(clawPos == 0) clawPos = 0.4;
                        else clawPos = 0;
                        clawServo.setPosition(clawPos);
                    }
                    clawOpen3 = gamepad2.right_bumper;
                })
                .transition(() -> {
                    boolean isAButtonPressed = gamepad2.a;
                    boolean fallingEdgeDetected = !wasAButtonPressed && isAButtonPressed;
                    wasAButtonPressed = isAButtonPressed;
                    return fallingEdgeDetected;
                }, ArmState.INTAKE_POSITION)
                .state(ArmState.AUTO_SUPER_SPEC)
                .loop(() -> {
                    currentState = ArmState.AUTO_SUPER_SPEC;
                    setArmPosition(640);
                    clawServo.setPosition(clawPos);
                    wristLeft.setPosition(1);
                    pivotServo.setPosition(0);
                    clawRotate.setPosition(0);
                    if(gamepad2.right_bumper && !clawOpen3){
                        if(clawPos == 0) clawPos = 0.4;
                        else clawPos = 0;
                        clawServo.setPosition(clawPos);
                    }
                    clawOpen3 = gamepad2.right_bumper;
                })
                .transition(() -> hi, ArmState.SPECIMEN1)
                .state(ArmState.AUTO_SPECIMEN_PICKUP1)
                .loop(() -> {
                    setArmPosition(100);
                    wristLeft.setPosition(1);
                    pivotServo.setPosition(0.5);
                    clawServo.setPosition(clawPos);
                    if(gamepad2.right_bumper && !clawOpen2){
                        if(clawPos == 0) clawPos = 0.5;
                        else clawPos = 0;
                        clawServo.setPosition(clawPos);
                    }
                    clawOpen2 = gamepad2.right_bumper;
                })
                .build();
    }






    public void setClawRotate(double i){
        clawRotate.setPosition(i);
    }
    public void setClawServo(double i){clawServo.setPosition(i);}
    public void setArmPosition(double targetPosition) {
        TARGET_POSITION = (int)targetPosition;
        /*int currentPosition = armControllerLeft.getCurrentPosition();
        double output = pidController.calculate(targetPosition, currentPosition);

        // Set motor power based on PID output
        armControllerLeft.setPower(output);
        armControllerRight.setPower(output);*/
    }
    public void setSlidesPos(double target){
        slidesTarget = (int)target;
    }

    public int getSlidesTarget() {
        return slidesTarget;
    }


 //   public void update() {
        // Placeholder for periodic updates if needed
//        double output = pidController.calculate(TARGET_POSITION, getArmPosition());
//        slidesPower = slidesController.calculate(slidesTarget,slidesMotor.getCurrentPosition());
//
//        // Set motor power based on PID output
//        slidesMotor.setPower(slidesPower);
//        armControllerLeft.setPower(output);
//        armControllerRight.setPower(output);

    //}
    public void update() {

        if(currentState == ArmState.HIGH_BASKET || currentState == ArmState.OUTTAKE_STAGE ||
        currentState == ArmState.AUTO_HIGH_BASKET_1 || currentState == ArmState.AUTO_HIGH_BASKET_2 ||
        currentState == ArmState.AUTO_SUPER_SPEC || currentState == ArmState.SUPER_SPEC) {
           slidesPower = slidesController.calculate(slidesTarget, slidesMotor.getCurrentPosition());
           slidesMotor.setPower(slidesPower);
       }

        // Arm control remains the same
        double output = pidController.calculate(TARGET_POSITION, getArmPosition());
        armControllerLeft.setPower(output);
        armControllerRight.setPower(output);
    }


    public void stopArm() {
        armControllerLeft.setPower(0);
        armControllerRight.setPower(0);
        slidesMotor.setPower(0);
        pidController.reset();
        slidesController.reset();// Reset PID controller
    }

    public int getArmPosition() {
        return armControllerLeft.getCurrentPosition();
    }
}
/*
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

@Config
public class ArmController {

    // Motors for the arm
    private DcMotorEx armControllerLeft;
    private DcMotorEx armControllerRight;

    private ArmState currentState;

    public PIDController pidController;

    // Servos
    private Servo wristLeft;

    private Servo wristRight;
    private Servo pivotServo;
    private Servo clawServo;
    private Servo clawRotate;

    // Positions (in encoder counts or degrees)
    public static int HIGH_BASKET_POSITION = 650;
    public static int LOW_BASKET_POSITION = 800;
    public static int HIGH_CHAMBER_POSITION = 1200;
    public static int INTAKE_POSITION = 150;
    public static int NEUTRAL_POSITION = 200;

    // Toggles for state transitions
    boolean highBasketToggle = false;
    boolean intakeStaging = false;

    public ArmController(HardwareMap hardwareMap) {
        // Initialize motors
        armControllerLeft = hardwareMap.get(DcMotorEx.class, "armMotorLeft");
        armControllerRight = hardwareMap.get(DcMotorEx.class, "armMotorRight");

        // Set motor behaviors
        armControllerLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        armControllerRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        armControllerLeft.setDirection(DcMotorEx.Direction.FORWARD); // Reversed
        armControllerRight.setDirection(DcMotorSimple.Direction.REVERSE); // Not reversed
        armControllerLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armControllerRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armControllerLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        armControllerRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize servos
        wristLeft = hardwareMap.get(Servo.class, "wristLeft");
        wristRight = hardwareMap.get(Servo.class, "wristRight");
        pivotServo = hardwareMap.get(Servo.class, "pivotServo");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        clawRotate = hardwareMap.get(Servo.class, "clawRotate");

        // Reversed wrist servos
        wristLeft.setDirection(Servo.Direction.REVERSE);
        clawServo.setDirection(Servo.Direction.FORWARD);
    }

    public void armInit() {
        // Set initial arm position to neutral or starting position
        currentState = ArmState.NEUTRAL_POSITION;
    }

    public void setClawRotate(double i) {
        pivotServo.setPosition(i);
    }

    public enum ArmState {
        HIGH_BASKET,
        INTAKE_POSITION,
        NEUTRAL_POSITION
    }

    public StateMachine armMachine(Gamepad gamepad1, Gamepad gamepad2) {
        return new StateMachineBuilder()
                .state(ArmState.NEUTRAL_POSITION)
                .loop(() -> {
                    setArmPosition(NEUTRAL_POSITION);
                    wristLeft.setPosition(0.0);
                    pivotServo.setPosition(0.0);  // Example pivot position
                    clawServo.setPosition(0.7);
                    if (gamepad2.a) highBasketToggle = true;
                    if (gamepad2.b) intakeStaging = true;
                })
                .transition(() -> !gamepad2.a && highBasketToggle, ArmState.HIGH_BASKET)
                .transition(() -> !gamepad2.b && intakeStaging, ArmState.INTAKE_POSITION)

                .state(ArmState.HIGH_BASKET)
                .loop(() -> {
                    setArmPosition(HIGH_BASKET_POSITION);
                    wristLeft.setPosition(1.0);
                    pivotServo.setPosition(1.0);  // Example pivot position
                    if(gamepad2.a) highBasketToggle = false;
                })
                .transition(() -> !gamepad2.a && !highBasketToggle, ArmState.NEUTRAL_POSITION)

                .state(ArmState.INTAKE_POSITION)
                .loop(() -> {
                    setArmPosition(INTAKE_POSITION);
                    wristLeft.setPosition(0.0);
                    pivotServo.setPosition(0.0);
                    clawServo.setPosition(1.0);  // Example pivot position
                    if (gamepad2.b) intakeStaging = false;
                })
                .transition(() -> !gamepad2.b && !intakeStaging, ArmState.NEUTRAL_POSITION)

                .build();
    }

    public void setArmPosition(double targetPosition) {
        // Directly setting the arm motor to the target position
        double output = pidController.calculate(targetPosition, getArmPosition());

        // Set motor power based on PID output
        armControllerLeft.setPower(output);
        armControllerRight.setPower(output);
    }

    public void stopArm() {
        armControllerLeft.setPower(0);
        armControllerRight.setPower(0);
    }

    public int getArmPosition() {
        return armControllerLeft.getCurrentPosition();
    }

    // Update method for periodic tasks like PID control, feedback, or state management
    public void update() {
        // Example: You could include logic for updating the motor powers based on feedback
        // E.g., implementing a PID controller, feedback loop, or stopping at target position
        if (armControllerLeft.getCurrentPosition() == armControllerLeft.getTargetPosition()) {
            // Stop motors if the arm has reached its target position
            stopArm();
        }
    }



    // Control the claw position (open or close) based on position input (0 = closed, 1 = open)
    public void clawControl(int position) {
        if (position == 0) {
            clawRotate.setPosition(1.0); // Closed position
        } else if (position == 1) {
            clawRotate.setPosition(0.0); // Open position
        }
    }
}
*/