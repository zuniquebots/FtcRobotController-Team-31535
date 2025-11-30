package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "PedroPathing1 Blue", group = "Pedro Pathing")
public class PedroPathingStarterBlue_Arav extends OpMode {

    private Follower follower;
    private int currentState;
    private AutonomousPaths paths;
    private ElapsedTime pauseTimer = new ElapsedTime();

    // ------------ CORRECTED POSES ------------
    public static final Pose START_POSE = new Pose(63, 9, Math.toRadians(90));
    public static final Pose SHOOT_POSE = new Pose(62, 20, Math.toRadians(65));

    // The robot will now turn to 0 degrees AS it drives to this pose.
    public static final Pose COLLECT_POSE1 = new Pose(42, 35, Math.toRadians(0));
    public static final Pose COLLECT_POSE2 = new Pose(42, 60, Math.toRadians(0));
    public static final Pose COLLECT_POSE3 = new Pose(42, 85, Math.toRadians(0));


    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(START_POSE);
        follower.setMaxPower(0.8);

        paths = new AutonomousPaths(follower);
        currentState = 0;
        telemetry.addData("Status", "Initialization Complete");
        telemetry.update();
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        pauseTimer.reset();
    }

    @Override
    public void loop() {
        follower.update();
        runStateMachine();

        telemetry.addData("Current State", currentState);
        telemetry.addData("Robot X", follower.getPose().getX());
        telemetry.addData("Robot Y", follower.getPose().getY());
        telemetry.addData("Robot Heading", Math.toDegrees(follower.getPose().getHeading()));

        if (currentState == 2) {
            telemetry.addData("Pausing", "%.1f / 5.0 seconds", pauseTimer.seconds());
        }
        telemetry.update();
    }

    public static class AutonomousPaths {
        public PathChain driveToShootPose;
        public PathChain driveToCollect1;
        public PathChain collect1ToShoot;
        public PathChain driveToCollect2;
        public PathChain collect2ToShoot;
        public PathChain driveToCollect3;
        public PathChain collect3ToShoot;


        public AutonomousPaths(Follower follower) {
            // Path from Start to initial Shoot Pose
            driveToShootPose = follower
                    .pathBuilder()
                    .addPath(new BezierLine(START_POSE, SHOOT_POSE))
                    .setLinearHeadingInterpolation(START_POSE.getHeading(), SHOOT_POSE.getHeading())
                    .build();

            // Path from Shoot Pose to first Collect Pose
            driveToCollect1 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(SHOOT_POSE, COLLECT_POSE1))
                    .setLinearHeadingInterpolation(SHOOT_POSE.getHeading(), COLLECT_POSE1.getHeading())
                    .build();

            // Path from first Collect Pose back to Shoot Pose
            collect1ToShoot = follower
                    .pathBuilder()
                    .addPath(new BezierLine(COLLECT_POSE1, SHOOT_POSE))
                    .setLinearHeadingInterpolation(COLLECT_POSE1.getHeading(), SHOOT_POSE.getHeading())
                    .build();

            // Path from Shoot Pose to second Collect Pose
            driveToCollect2 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(SHOOT_POSE, COLLECT_POSE2))
                    .setLinearHeadingInterpolation(SHOOT_POSE.getHeading(), COLLECT_POSE2.getHeading())
                    .build();

            // Path from second Collect Pose back to Shoot Pose
            collect2ToShoot = follower
                    .pathBuilder()
                    .addPath(new BezierLine(COLLECT_POSE2, SHOOT_POSE))
                    .setLinearHeadingInterpolation(COLLECT_POSE2.getHeading(), SHOOT_POSE.getHeading())
                    .build();

            // Path from Shoot Pose to third Collect Pose
            driveToCollect3 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(SHOOT_POSE, COLLECT_POSE3))
                    .setLinearHeadingInterpolation(SHOOT_POSE.getHeading(), COLLECT_POSE3.getHeading())
                    .build();

            // Path from third Collect Pose back to Shoot Pose
            collect3ToShoot = follower
                    .pathBuilder()
                    .addPath(new BezierLine(COLLECT_POSE3, SHOOT_POSE))
                    .setLinearHeadingInterpolation(COLLECT_POSE3.getHeading(), SHOOT_POSE.getHeading())
                    .build();
        }
    }

    private void runStateMachine() {
        switch (currentState) {
            case 0: // Start driving to the first shoot position
                follower.followPath(paths.driveToShootPose);
                currentState = 1;
                break;
            case 1: // Wait, then pause
                if (!follower.isBusy()) {
                    pauseTimer.reset();
                    currentState = 2;
                }
                break;
            case 2: // Pause (e.g., for shooting)
                if (pauseTimer.seconds() >= 5.0) {
                    currentState = 3;
                }
                break;
            case 3: // Drive to the first collection point
                follower.followPath(paths.driveToCollect1);
                currentState = 4;
                break;
            case 4: // Wait, then drive back to shoot
                if (!follower.isBusy()) {
                    follower.followPath(paths.collect1ToShoot);
                    currentState = 5;
                }
                break;
            case 5: // Wait, then drive to the second collection point
                if (!follower.isBusy()) {
                    follower.followPath(paths.driveToCollect2);
                    currentState = 6;
                }
                break;
            case 6: // Wait, then drive back to shoot
                if (!follower.isBusy()) {
                    follower.followPath(paths.collect2ToShoot);
                    currentState = 7;
                }
                break;
            case 7: // Wait, then drive to the third collection point
                if (!follower.isBusy()) {
                    follower.followPath(paths.driveToCollect3);
                    currentState = 8;
                }
                break;
            case 8: // Wait, then drive back to shoot for the final time
                if (!follower.isBusy()) {
                    follower.followPath(paths.collect3ToShoot);
                    currentState = 9;
                }
                break;
            case 9: // Wait for the final path to complete
                if (!follower.isBusy()) {
                    currentState = 10;
                }
                break;
            case 10: // Autonomous is complete
                break;
        }
    }
}
