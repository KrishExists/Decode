package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.StateMachine;

/**
 * Intake subsystem that encapsulates intake motor, blocker servo, and the intake-related
 * state machine logic. It delegates shooter control to the Shooter subsystem when needed.
 */
public class Intake {
    public enum IntakeState { INTAKE, OUTTAKE, RUNSLOW, OUTTAKE1, OUTTAKE_FAR, OUTTAKE_MID, TRANSFER, REST }

    private final Hardware hw;
    private final StateMachine<IntakeState> sm;
    private final Shooter shooter;

    public Intake(Hardware hardware, Shooter shooter) {
        this.hw = hardware;
        this.sm = new StateMachine<>(IntakeState.REST);
        this.shooter = shooter;
    }

    public void setState(IntakeState s) { sm.setState(s); }
    public IntakeState getState() { return sm.getState(); }

    /**
     * Call regularly from an OpMode loop. Gamepad mapping follows the original TeleOp logic.
     */
    public void update(Gamepad g) {
        // map buttons to states (same logic as original)
        if (g.right_trigger > 0.2) setState(IntakeState.INTAKE);
        else if (g.dpad_left) setState(IntakeState.TRANSFER);
        else if (g.left_trigger > 0.2) setState(IntakeState.OUTTAKE);
        else if (g.right_bumper) setState(IntakeState.RUNSLOW);
        else if (g.a) setState(IntakeState.OUTTAKE1);
        else if (g.x) setState(IntakeState.OUTTAKE_FAR);
        else if (g.y) setState(IntakeState.OUTTAKE_MID);
        else setState(IntakeState.REST);

        // run state actions
        switch (sm.getState()) {
            case INTAKE:
                hw.intake.setPower(Constants.INTAKE_IN_POWER);
                shooter.stop();
                hw.linkage.setPosition(Constants.LINKAGE_REST);
                hw.blocker.setPosition(Constants.BLOCKER_OPEN);
                break;

            case TRANSFER:
                hw.blocker.setPosition(Constants.BLOCKER_CLOSE);
                hw.intake.setPower(Constants.INTAKE_REVERSE_TRANSFER);
                shooter.stop();
                break;

            case OUTTAKE1:
                hw.blocker.setPosition(Constants.BLOCKER_OPEN);
                if (sm.timeInStateMs() < 1500) {
                    shooter.spin(2000);
                    hw.intake.setPower(0);
                    hw.linkage.setPosition(Constants.LINKAGE_REST);
                } else if (sm.timeInStateMs() < 3300) {
                    shooter.spin(2400);
                    hw.intake.setPower(0);
                    hw.linkage.setPosition(Constants.LINKAGE_SHOOT);
                } else {
                    shooter.spin(2400);
                    hw.intake.setPower(Constants.INTAKE_FEED_POWER);
                    hw.linkage.setPosition(Constants.LINKAGE_SHOOT);
                }
                break;

            case OUTTAKE:
                hw.blocker.setPosition(Constants.BLOCKER_OPEN);
                if (sm.timeInStateMs() < 700) {
                    hw.intake.setPower(-0.5);
                    shooter.reverse();
                    hw.linkage.setPosition(Constants.LINKAGE_REST);
                } else if (sm.timeInStateMs() < 2500) {
                    shooter.spin(2700);
                    hw.intake.setPower(0);
                    hw.linkage.setPosition(Constants.LINKAGE_REST);
                } else if (sm.timeInStateMs() < 3000) {
                    shooter.spin(2700);
                    hw.intake.setPower(0);
                    hw.linkage.setPosition(Constants.LINKAGE_SHOOT);
                } else if (sm.timeInStateMs() < 4000) {
                    shooter.spin(2700);
                    hw.intake.setPower(Constants.INTAKE_FEED_POWER);
                    hw.linkage.setPosition(Constants.LINKAGE_SHOOT);
                } else {
                    shooter.spin(2700);
                    hw.intake.setPower(Constants.INTAKE_FEED_POWER);
                }
                break;

            case OUTTAKE_MID:
                hw.blocker.setPosition(Constants.BLOCKER_OPEN);
                if (sm.timeInStateMs() < 500) {
                    hw.linkage.setPosition(Constants.LINKAGE_MID);
                } else {
                    shooter.spin(3500);
                    if (shooter.atSpeed(3400, 3600)) hw.intake.setPower(Constants.INTAKE_FEED_POWER);
                    else hw.intake.setPower(0);
                }
                break;

            case OUTTAKE_FAR:
                hw.blocker.setPosition(Constants.BLOCKER_OPEN);
                if (sm.timeInStateMs() < 500) {
                    hw.linkage.setPosition(Constants.LINKAGE_MID);
                } else {
                    shooter.spin(6000);
                    if (shooter.atSpeed(3700, 6000)) hw.intake.setPower(Constants.INTAKE_FEED_POWER);
                    else hw.intake.setPower(0);
                }
                break;

            case RUNSLOW:
                hw.intake.setPower(-1.0);
                shooter.reverse();
                hw.linkage.setPosition(Constants.LINKAGE_REST);
                break;

            default: // REST
                hw.intake.setPower(0.0);
                shooter.stop();
                hw.blocker.setPosition(Constants.BLOCKER_CLOSE);
                hw.linkage.setPosition(Constants.LINKAGE_REST);
                break;
        }
    }
}
