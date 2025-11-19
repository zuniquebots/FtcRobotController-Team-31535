package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants; // Replace with your actual Constants file

@Autonomous(name = "Far Shot Auto", group = "Pedro Pathing")
public class FarShotAuto extends LinearOpMode {

    // Define the initial and final Poses
    private final Pose START_POSE = new Pose(0, 72, 0);
    private final Pose FAR_SHOT_POSE = new Pose(96, 120, 90);

    @Override
    public void runOpMode() {
        // *** STEP 1: INITIALIZE THE FOLLOWER ***
        // Use your team's specific Constants class or initialization method
        Follower follower = Constants.createFollower(hardwareMap);

        // *** STEP 2: SET THE INITIAL ROBOT POSE ***
        follower.setStartingPose(START_POSE);

        // *** STEP 3: WAIT FOR START ***
        telemetry.addData("Status", "Initialized. Start Pose: " + START_POSE.toString());
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        // *** STEP 4: BUILD AND FOLLOW THE PATH ***
        try {
            // Build the straight path
            PathChain farShotPath = follower.pathBuilder()
                    // Create the straight line segment
                    .addPath(new BezierLine(START_POSE, FAR_SHOT_POSE))
                    // Define the heading change over the path
                    .setLinearHeadingInterpolation(START_POSE.getHeading(), FAR_SHOT_POSE.getHeading())
                    .build();

            // Execute the path
            follower.followPath(farShotPath);

            // Loop while the path is running
            while (opModeIsActive() && follower.isBusy()) {
                follower.update();
                telemetry.addData("Status", "Following Path...");
                telemetry.addData("Current Pose", follower.getPose().toString());
                telemetry.update();
            }
        } catch (Exception e) {
            telemetry.addData("ERROR", "Path execution failed: " + e.getMessage());
            telemetry.update();
            sleep(5000);
        }

        telemetry.addData("Status", "Path Complete!");
        telemetry.update();
    }
}