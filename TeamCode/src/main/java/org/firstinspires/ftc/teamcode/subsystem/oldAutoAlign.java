
//    private double normalize(double angle) {
//        while (angle > Math.PI) angle -= 2 * Math.PI;
//        while (angle < -Math.PI) angle += 2 * Math.PI;
//        return angle;
//    }
//
//
//    private double angleToTarget(Pose2d current, Pose2d target) {
//        double dx = target.position.x - current.position.x;
//        double dy = target.position.y - current.position.y;
//        return Math.atan2(dy, dx);
//    }
//
//    public void alignToPose(Pose2d target) {
//        Pose2d current = currentPose;
//
//        // Phase 1: rotate to face the target XY
//        double angleToPoint = angleToTarget(current, target);
//        double turnToFace = normalize(angleToPoint - current.heading.toDouble());
//
//        // Phase 2: rotate to match final heading
//        double turnToFinal = normalize(target.heading.toDouble() - current.heading.toDouble());
//
//        double kP = 1.0;   // recommended tuning
//
//        // If not facing the target point yet → turn toward it
//        if (Math.abs(turnToFace) > Math.toRadians(3)) {
//            double turn = kP * turnToFace;
//            turn = Math.max(-0.6, Math.min(0.6, turn));  // clamp
//
//            setMotorPowers(
//                    turn,  // left front
//                    -turn, // right front
//                    turn,  // left back
//                    -turn  // right back
//            );
//            return;
//        }
//
//        // Otherwise, match target heading
//        double finalTurn = kP * turnToFinal;
//        finalTurn = Math.max(-0.6, Math.min(0.6, finalTurn));
//
//        setMotorPowers(
//                finalTurn,
//                -finalTurn,
//                finalTurn,
//                -finalTurn
//        );
//    }


