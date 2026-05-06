package org.firstinspires.ftc.teamcode.subsystems.shooting;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public interface ShotCalculator {
    void init();

    ShotSolution run();

    void reset();

    void updateRobotPose(Pose2D robotPose);

    void updateGoalPose(Pose2D goalPose);

    default void updateRobotVelocity(double vx, double vy, double omega) {
        // optional
    }
}