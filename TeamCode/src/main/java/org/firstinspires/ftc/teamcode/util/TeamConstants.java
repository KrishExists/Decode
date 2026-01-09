package org.firstinspires.ftc.teamcode.util;

public class TeamConstants {

    // =============================
    // Intake Constants
    // =============================
    public static final double INTAKE_IN_POWER = 0.8;
    public static final double INTAKE_FEED_POWER = 1.0;
    public static final double INTAKE_EVEN_POWER = 0.6;
    public static final double INTAKE_DEFAULT_POWER = 0.0;
    public static final double intakeReversed = -1;


    // =============================
    // Transfer / Outtake Constants
    // =============================
    public static final double TRANSFER_IN_POWER = -1;
    public static final double TRANSFER_EVEN = 0.8;
    public static final double TRANSFER_CLOSED = 0;
    public static final double TRANSFER_REV = 1;

    public static final double outtake_Stop = 0;
    public static final double SLIGHT_REVERSE_OUTTAKE = -0.8;


    // =============================
    // Linkage Servo Positions
    // =============================
    public static final double LINKAGE_REST = 0.5;
    public static final double LINKAGE_SHOOT = 0.47;


    // =============================
    // Shooter RPM Targets
    // =============================
    public static final double SHOOTER_MID_RPM = 2500;
    public static final double SHOOTER_FAR_RPM = 3200;

    public static final double Shooter_BottomThreshold = 2300;

    public static final double SHOOTER_CLOSED = 0;


    // =============================
    // Shooter PID Constants
    // =============================
    public static final double SHOOTER_kP = 0.02;
    public static final double SHOOTER_kI = 0.0;
    public static final double SHOOTER_kD = 0.0003;
    public static double lastError = 0;
    public static double integral = 0;


    // =============================
    // Sensors
    // =============================
    public static final double BALL_THRESHOLD_CM = 5.0;


}
