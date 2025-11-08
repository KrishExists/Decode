package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.Intake;
import org.firstinspires.ftc.teamcode.subsystem.Outtake;

/**
 * DosaController – coordinates the Outtake and Intake systems
 * Handles timing, sequencing, and mode transitions.
 */
public class DosaController implements Control {

    private final Intake intake;
    private final Outtake outtake;
    private final Telemetry telemetry;

    private final ElapsedTime timer = new ElapsedTime();

    public enum State {
        INTAKE,
        OUTTAKE,
        RUNSLOW,
        OUTTAKE1,
        REST
    }

    private State currentState = State.REST;
    private State previousState = State.REST;

    public DosaController(Intake intake, Outtake outtake, Telemetry telemetry) {
        this.intake = intake;
        this.outtake = outtake;
        this.telemetry = telemetry;
    }

    public void setState(State newState) {
        this.currentState = newState;
    }

    public State getState() {
        return currentState;
    }

    @Override
    public void update() {
        if (currentState != previousState) {
            timer.reset();
            previousState = currentState;
        }

        switch (currentState) {
            case INTAKE:
                intake.setPower(0.8);
                outtake.spinToRpm(-1000);
                outtake.setLinkage(0.92);
                break;

            case OUTTAKE1:
                if (timer.milliseconds() < 1500) {
                    outtake.spinToRpm(4000);
                    intake.setPower(0.0);
                    outtake.setLinkage(0.92);
                } else if (timer.milliseconds() < 3300) {
                    outtake.spinToRpm(4000);
                    outtake.setLinkage(0.25);
                    intake.setPower(0.0);
                } else {
                    outtake.spinToRpm(4000);
                    outtake.setLinkage(0.25);
                    intake.setPower(0.8);
                }
                break;

            case OUTTAKE:
                if (timer.milliseconds() < 700) {
                    outtake.spinToRpm(4000);
                    intake.setPower(-0.5);
                    outtake.setLinkage(0.92);
                } else if (timer.milliseconds() < 3500) {
                    outtake.spinToRpm(4000);
                    outtake.setLinkage(0.25);
                    intake.setPower(0.0);
                } else {
                    outtake.spinToRpm(4000);
                    outtake.setLinkage(0.25);
                    intake.setPower(0.8);
                }
                break;

            case RUNSLOW:
                intake.setPower(0.4);
                outtake.spinToRpm(0.0);
                outtake.setLinkage(0.92);
                break;

            case REST:
                intake.setPower(0.0);
                outtake.spinToRpm(0.0);
                outtake.setLinkage(0.92);
                break;
        }

        telemetry.addData("Dosa State", currentState);
        telemetry.addData("Timer", timer.milliseconds());
        telemetry.update();
    }
}
