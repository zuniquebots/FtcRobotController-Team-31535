package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants1; // IMPORTANT: Use your actual Constants file name

/**
 * A starter autonomous OpMode for Pedro Pathing.
 * This OpMode drives the robot forward 24 inches from a defined starting position.
 */
@Autonomous(name = "Pedro Pathing Starter 1", group = "Pedro Pathing")
public class PedroPathingStarter1 extends OpMode {

    // Pedro Pathing follower object
    private Follower follower;

    // State machine for autonomous sequence
    private int currentState;

    // A class to hold all our predefined paths
    private AutonomousPaths paths;

    // ------------ DEFINE YOUR POSES HERE ------------
    // Define the robot's starting position on the field.
    // X, Y are in inches. Heading is in radians.
    // Example: Start at X=12, Y=64, facing forward (90 degrees or PI/2 radians).
    public static final Pose START_POSE = new Pose(64, 0, Math.toRadians(0));

    // Define the target position, 24 inches "forward" from the start.
    // Since the robot is facing 90 degrees, "forward" means increasing the Y coordinate.
    public static final Pose FORWARD_POSE = new Pose(48,90 , Math.toRadians(0));

    /**
     * This method is called once when the "INIT" button is pressed.
     */
    @Override
    public void init() {
        // IMPORTANT: Initialize the Pinpoint localizer.
        // The name "PinpointComputer" must match your robot's hardware configuration.
        // This line creates a new driver instance, which is all that's needed.

        // Create the follower using your Constants file.
        // Ensure 'Constants.java' is correctly configured.
        follower = Constants1.createFollower(hardwareMap);

        // Tell the follower where the robot is starting on the field.
        follower.setStartingPose(START_POSE);

        // Create an object that holds all of our paths.
        paths = new AutonomousPaths(follower);

        // Initialize the state machine to its first state.
        currentState = 0;

        telemetry.addData("Status", "Initialization Complete");
        telemetry.update();
    }

    /**
     * This method is called repeatedly after "INIT" is pressed but before "START".
     */
    @Override
    public void init_loop() {
        // You can add things here to run during the init phase, like vision processing.
    }

    /**
     * This method is called once when the "START" button is pressed.
     */
    @Override
    public void start() {
        // Code here will run once at the beginning of the autonomous period.
    }

    /**
     * This method is called repeatedly during the autonomous period.
     */
    @Override
    public void loop() {
        // Main autonomous loop.

        // 1. Update the follower. This reads the latest position and calculates motor powers.
        follower.update();

        // 2. Run our state machine.
        runStateMachine();

        // 3. Display useful information on the Driver Station.
        telemetry.addData("Current State", currentState);
        telemetry.addData("Robot X", follower.getPose().getX());
        telemetry.addData("Robot Y", follower.getPose().getY());
        telemetry.addData("Robot Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    /*
     * This method is called once when the OpMode is stopped.
     */

    /**
     * This class holds all the PathChain objects for our autonomous routine.
     * This keeps the main OpMode file clean and organized.
     */
    public static class AutonomousPaths {
        public PathChain driveForwardPath;
        // You can add more paths here, e.g., public PathChain scoreOnBackdrop;

        public AutonomousPaths(Follower follower) {
            // Build the path to drive forward 24 inches.
            driveForwardPath = follower
                    .pathBuilder()
                    .addPath(new BezierLine(START_POSE, FORWARD_POSE))
                    // This keeps the robot's heading constant throughout the movement.
                    .setLinearHeadingInterpolation(START_POSE.getHeading(), FORWARD_POSE.getHeading())
                    .build();

            // Build other paths here...
        }
    }

    /**
     * A simple state machine to manage the sequence of actions.
     */
    private void runStateMachine() {
        switch (currentState) {
            case 0:
                // State 0: Start following the first path.
                follower.followPath(paths.driveForwardPath);

                // Immediately advance to the next state to wait for completion.
                currentState = 1;
                break;

            case 1:
                // State 1: Wait for the current path to finish.
                if (!follower.isBusy()) {
                    // The robot has reached its destination.
                    // Advance to the "finished" state.
                    currentState = 2;
                }
                // If it's still busy, do nothing and stay in this state.
                break;

            case 2:
                // State 2: Autonomous routine is complete.
                // The follower will automatically hold the robot's final position.
                // You could add more states to drive another path.
                break;
        }
    }
}
