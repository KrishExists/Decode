package org.firstinspires.ftc.teamcode.PedroHelper;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

public class DriveStep implements Step {

    // ── deps ──────────────────────────────────────────────────────────────────

    private final Follower  follower;
    private final PathChain pathChain;  // one of these will be set
    private final Path      path;

    // ── hooks ─────────────────────────────────────────────────────────────────

    Runnable onStart;
    Runnable onComplete;
    Runnable whileDriving;
    List<PercentMarker> percentMarkers = new ArrayList<>();

    // ── runtime state ─────────────────────────────────────────────────────────

    private boolean   completeFired   = false;
    private boolean[] markerTriggered;

    // ── constructors ──────────────────────────────────────────────────────────

    public DriveStep(Follower follower, PathChain pathChain) {
        this.follower  = follower;
        this.pathChain = pathChain;
        this.path      = null;
    }

    public DriveStep(Follower follower, Path path) {
        this.follower  = follower;
        this.path      = path;
        this.pathChain = null;
    }

    // ── Step impl ─────────────────────────────────────────────────────────────

    @Override
    public void onEnter() {
        completeFired = false;

        // sort markers ascending so we can walk them in order
        percentMarkers.sort(Comparator.comparingDouble(m -> m.percent));
        markerTriggered = new boolean[percentMarkers.size()];

        if (pathChain != null) {
            follower.followPath(pathChain, true);
        } else {
            follower.followPath(path, true);
        }

        if (onStart != null) onStart.run();
    }

    @Override
    public void execute() {
        // per-tick continuous action
        if (whileDriving != null) whileDriving.run();

        // percent markers — follower.getCurrentTValue() returns 0.0–1.0
        double t = follower.getCurrentTValue();
        for (int i = 0; i < percentMarkers.size(); i++) {
            if (!markerTriggered[i] && t >= percentMarkers.get(i).percent) {
                percentMarkers.get(i).action.run();
                markerTriggered[i] = true;
            }
        }
    }

    @Override
    public boolean isComplete() {
        if (!follower.isBusy()) {
            if (!completeFired) {
                completeFired = true;

                // fire any markers that were never reached (e.g. path ended early)
                for (int i = 0; i < percentMarkers.size(); i++) {
                    if (!markerTriggered[i]) {
                        percentMarkers.get(i).action.run();
                        markerTriggered[i] = true;
                    }
                }

                if (onComplete != null) onComplete.run();
            }
            return true;
        }
        return false;
    }
}