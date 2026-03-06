package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ShootWhileMoving {

    // Tune these for your robot
    static final double PROJECTILE_SPEED = 284.0; // inches per second

    /**
     * Call this in your OpMode loop to continuously aim and shoot while moving.
     * It will call shootFromPose() with a velocity-compensated virtual goal.
     */
    public static Pose getCompensatedGoal(Follower follower, Pose realGoal) {

        Pose currentPose = follower.getPose();
        Vector velocity = follower.getVelocity();
        // --- First Pass ---
        double dx = realGoal.getX() - currentPose.getX();
        double dy = realGoal.getY() - currentPose.getY();
        double realDistance = Math.sqrt(dx * dx + dy * dy);
        realDistance = currentPose.distanceFrom(realGoal);
        double flightTime = realDistance / PROJECTILE_SPEED;

        Pose virtualGoal = new Pose(
                realGoal.getX() - velocity.getXComponent() * flightTime,
                realGoal.getY() - velocity.getYComponent() * flightTime
        );

        // --- Second Pass ---
        double dx2 = virtualGoal.getX() - currentPose.getX();
        double dy2 = virtualGoal.getY() - currentPose.getY();
        double refinedDistance = Math.sqrt(dx2 * dx2 + dy2 * dy2);
        refinedDistance = currentPose.distanceFrom(virtualGoal);
        double refinedFlightTime = refinedDistance / PROJECTILE_SPEED;

        Pose refinedVirtualGoal = new Pose(
                realGoal.getX() - velocity.getXComponent() * refinedFlightTime,
                realGoal.getY() - velocity.getYComponent() * refinedFlightTime
        );

        return refinedVirtualGoal;
    }
}