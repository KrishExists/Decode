package org.firstinspires.ftc.teamcode.util;

public class TeamConstants {

    // =============================
    // Intake Constants
    // =============================
    public static final double INTAKE_IN_POWER = 1.0;
    public static final double INTAKE_FEED_POWER = 1.0;
    public static final double INTAKE_EVEN_POWER = 0.6;
    public static final double INTAKE_DEFAULT_POWER = 0.0;
    public static final double intakeReversed = -1;
    public static final double BLOCKER_CLOSE = 0.55;
    public static final double BLOCKER_OPEN = 0.58;

    // =============================
    // Transfer / Outtake Constants
    // =============================
    public static final double TRANSFER_IN_POWER = 1;
    public static final double TRANSFER_IN_POWER_AUTO = 0.8;

    public static final double TRANSFER_EVEN = -0.8;
    public static final double TRANSFER_CLOSED = 0;
    public static final double TRANSFER_REVEVERSED = -1;

    public static final double outtake_Stop = 0;
    public static final double SLIGHT_REVERSE_OUTTAKE = -0.8;


    // =============================
    // Linkage Servo Positions
    // =============================
    public static final double LINKAGE_REST = 1;
    public static final double LINKAGE_SHOOT = 1;


    // =============================
    // Shooter RPM Targets
    // =============================
    public static final double SHOOTER_MID_RPM = 3900;
    public static final double SHOOTER_FAR_RPM = 2850;

    public static final double SHOOTER_FARAUTO_RPM = 2950;

    public static final double Shooter_BottomThreshold = 2300;

    public static final double SHOOTER_CLOSED = 0;


    // =============================
    // Shooter PID Constants
    // =============================

    public static double lastError = 0;
    public static double integral = 0;


    // =============================
    // Sensors
    // =============================
    public static final double BALL_THRESHOLD_CM = 5.0;


}
