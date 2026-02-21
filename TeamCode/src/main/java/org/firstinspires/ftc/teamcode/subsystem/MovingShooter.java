package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.roadrunner.Pose2d.*;

import org.firstinspires.ftc.teamcode.util.ShooterConstants;
import org.firstinspires.ftc.teamcode.util.TeamConstants;

public class MovingShooter extends ShooterConstants {
    // ---- FLYWHEEL ----
    private static final double FLYWHEEL_DIAMETER_METERS = 0.072;
    // ^ 72mm flywheel (CHANGE if different)
    private static final double TICKS_PER_REV = 28;
    // ^ CHANGE if your encoder has different CPR

    private static final double BASE_RPM_CLOSE = (2100);
    // ^ Tune for ~60 inch shots

    private static final double BASE_RPM_MID = (TeamConstants.SHOOTER_MID_RPM);
    // ^ Tune for ~65–70 inch shots

    private static final double BASE_RPM_FAR = (TeamConstants.SHOOTER_FAR_RPM);
    // ^ Tune for ~75 inch shots

    // ---- FIELD COORDINATES ----
    private static final double GOAL_X = 133;

    private static final double GOAL_Y = 133;

    // ---- MOTION COMPENSATION ----
    private static final double TIME_OF_FLIGHT = 0.18;
   //Tune between 0.15–0.25 if shots missed

    private static final double INCHES_TO_METERS = 0.0254;

    private DcMotorEx shooter1;
    private DcMotorEx shooter2;

    private final double wheelCircumference;

    // ==============================
    // ===== CONSTRUCTOR ============
    // ==============================

    public MovingShooter(HardwareMap hardwareMap) {

        shooter1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");

        // Optional: reverse one motor if needed
        shooter2.setDirection(DcMotorEx.Direction.REVERSE);

        double radius = FLYWHEEL_DIAMETER_METERS / 2.0;
        wheelCircumference = 2.0 * Math.PI * radius;
    }

    // ==============================
    // ===== MAIN SHOOT METHOD ======
    // ==============================

    public void shootWhileMoving(Pose2d pose, Pose2d velocity) {

        // ------------------------------
        // 1. Calculate distance to goal
        // ------------------------------

        double dx = GOAL_X - pose.position.x;
        double dy = GOAL_Y - pose.position.y;

        double distance = Math.hypot(dx, dy);

        // ------------------------------
        // 2. Get base RPM from distance
        // ------------------------------

        double baseRPM = getRPMFromDistance(distance);
        // 🔧 Tune RPM values in that method

        // Convert base RPM to exit velocity (m/s)
        double baseVelocity = (baseRPM / 60.0) * wheelCircumference;

        // ------------------------------
        // 3. Robot velocity (in/s → m/s)
        // ------------------------------

        double robotVX = (velocity.position.x);
        double robotVY = (velocity.position.y);

        // ------------------------------
        // 4. Predictive offset (lead)
        // ------------------------------

        dx += (velocity.position.x) * TIME_OF_FLIGHT;
        dy += (velocity.position.y) * TIME_OF_FLIGHT;
        // 🔥 Tune TIME_OF_FLIGHT if missing while moving

        double targetAngle = Math.atan2(dy, dx);

        // ------------------------------
        // 5. Desired ball vector
        // ------------------------------

        double desiredVX = baseVelocity * Math.cos(targetAngle);
        double desiredVY = baseVelocity * Math.sin(targetAngle);

        // ------------------------------
        // 6. Subtract robot motion
        // ------------------------------

        double compensatedVX = desiredVX - robotVX;
        double compensatedVY = desiredVY - robotVY;

        double compensatedSpeed = Math.hypot(compensatedVX, compensatedVY);

        // ------------------------------
        // 7. Convert back to RPM
        // ------------------------------

        double newRPM = (compensatedSpeed / wheelCircumference) * 60.0;

        // ------------------------------
        // 8. Apply to motors
        // ------------------------------

        double ticksPerSecond = rpmToTicksPerSecond(newRPM);

        shooter1.setVelocity(ticksPerSecond);
        shooter2.setVelocity(ticksPerSecond);
    }

    // ==============================
    // ===== DISTANCE → RPM TABLE ===
    // ==============================

    private double getRPMFromDistance(double distance) {

        // 🔧 Adjust these thresholds after testing

        if (distance < 65) {
            return BASE_RPM_CLOSE;
        }
        else if (distance < 72) {
            return BASE_RPM_MID;
        }
        else {
            return BASE_RPM_FAR;
        }
    }

    // ==============================
    // ===== RPM → TICKS ============
    // ==============================

    private double rpmToTicksPerSecond(double rpm) {
        return (rpm * TICKS_PER_REV) / 60.0;
    }

    // ==============================
    // ===== STOP SHOOTER ===========
    // ==============================

}
