package org.firstinspires.ftc.teamcode.PedroHelper;

public class PercentMarker {
    public final double percent;   // 0.0 – 1.0
    public final Runnable action;

    public PercentMarker(double percent, Runnable action) {
        if (percent < 0.0 || percent > 1.0) {
            throw new IllegalArgumentException(
                    "PercentMarker percent must be between 0.0 and 1.0, got: " + percent
            );
        }
        this.percent = percent;
        this.action  = action;
    }
}