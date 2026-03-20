package org.firstinspires.ftc.teamcode.PedroHelper;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.PedroHelper.AutoRoutine;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

public class AutoBuilder {

    private final Follower   follower;
    private final List<Step> steps = new ArrayList<>();

    // pending DriveStep — held here until flushed by the next non-modifier call
    private DriveStep pendingDrive = null;

    public AutoBuilder(Follower follower) {
        this.follower = follower;
    }

    // =========================================================================
    // Setup
    // =========================================================================

    public AutoBuilder startAt(Pose pose) {
        follower.setStartingPose(pose);
        return this;
    }

    // =========================================================================
    // Drive step
    // =========================================================================

    public AutoBuilder drive(PathChain path) {
        flushPending();
        pendingDrive = new DriveStep(follower, path);
        return this;
    }

    public AutoBuilder drive(Path path) {
        flushPending();
        pendingDrive = new DriveStep(follower, path);
        return this;
    }

    // =========================================================================
    // Drive modifiers — must follow a .drive() call
    // =========================================================================

    public AutoBuilder onStart(Runnable action) {
        requirePendingDrive("onStart");
        pendingDrive.onStart = action;
        return this;
    }

    public AutoBuilder onComplete(Runnable action) {
        requirePendingDrive("onComplete");
        pendingDrive.onComplete = action;
        return this;
    }

    public AutoBuilder whileDriving(Runnable action) {
        requirePendingDrive("whileDriving");
        pendingDrive.whileDriving = action;
        return this;
    }

    public AutoBuilder atPercent(double percent, Runnable action) {
        requirePendingDrive("atPercent");
        pendingDrive.percentMarkers.add(new PercentMarker(percent, action));
        return this;
    }

    // =========================================================================
    // Non-drive steps — each flushes pendingDrive first
    // =========================================================================

    public AutoBuilder waitMs(long ms) {
        flushPending();
        steps.add(new WaitMsStep(ms));
        return this;
    }

    public AutoBuilder waitUntil(BooleanSupplier condition) {
        flushPending();
        steps.add(new WaitUntilStep(condition));
        return this;
    }

    public AutoBuilder runOnce(Runnable action) {
        flushPending();
        steps.add(new RunOnceStep(action));
        return this;
    }

    public AutoBuilder stop() {
        flushPending();
        steps.add(new StopStep());
        return this;
    }

    // =========================================================================
    // Build
    // =========================================================================

    public AutoRoutine build() {
        flushPending();
        return new AutoRoutine(follower, new ArrayList<>(steps));
    }

    // =========================================================================
    // Internal helpers
    // =========================================================================

    /** Push pendingDrive into the step list if one is waiting. */
    private void flushPending() {
        if (pendingDrive != null) {
            steps.add(pendingDrive);
            pendingDrive = null;
        }
    }

    /** Throw a clear error if a modifier is called with no preceding drive(). */
    private void requirePendingDrive(String methodName) {
        if (pendingDrive == null) {
            throw new IllegalStateException(
                    "AutoBuilder." + methodName + "() called with no preceding drive(). "
                            + "Call drive(PathChain) first."
            );
        }
    }
}