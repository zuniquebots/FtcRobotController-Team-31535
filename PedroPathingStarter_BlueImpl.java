package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo; // Import the Servo class
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "PedroPathingBlueImpl", group = "Pedro Pathing")
public class PedroPathingStarter_BlueImpl extends OpMode {

    private Follower follower;
    private int currentState;
    private AutonomousPaths paths;
    private ElapsedTime pauseTimer = new ElapsedTime();

    // Hardware
    private DcMotor launchMotor;
    private DcMotor leftIntake;
    private DcMotor rightIntake;
    private Servo rightServo; // Added servo for shooting
    private Servo leftServo;  // Added servo for shooting


    // ------------ CORRECTED POSES ------------
    public static final Pose START_POSE = new Pose(63, 9, Math.toRadians(90));
    public static final Pose SHOOT_POSE = new Pose(61, 17, Math.toRadians(65));

    // The robot will now turn to 0 degrees AS it drives to this pose.
    public static final Pose COLLECT_POSE1 = new Pose(42, 35, Math.toRadians(0));
    public static final Pose COLLECT_POSE2 = new Pose(42, 60, Math.toRadians(0));
    public static final Pose COLLECT_POSE3 = new Pose(42, 85, Math.toRadians(0));


    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(START_POSE);
        follower.setMaxPower(0.8);

        // --- Initialize Hardware ---
        launchMotor = hardwareMap.get(DcMotor.class, "launchMotor");
        leftIntake = hardwareMap.get(DcMotor.class, "leftIntakeMotor");
        rightIntake = hardwareMap.get(DcMotor.class, "rightIntakeMotor");
        rightServo = hardwareMap.get(Servo.class, "rightServo");
        leftServo = hardwareMap.get(Servo.class, "leftServo");


        // --- Set Motor and Servo Directions ---
        launchMotor.setDirection(DcMotor.Direction.FORWARD);
        rightIntake.setDirection(DcMotor.Direction.FORWARD);
        leftIntake.setDirection(DcMotor.Direction.FORWARD);
        rightServo.setDirection(Servo.Direction.REVERSE);
        leftServo.setDirection(Servo.Direction.FORWARD);


        paths = new AutonomousPaths(follower);
        currentState = 0;
        // Set servos to initial position (holding the ball)
        rightServo.setPosition(0);
        leftServo.setPosition(0);
        telemetry.addData("Status", "Initialization Complete1");
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

        if (currentState == 2 || currentState == 6 || currentState == 9 || currentState == 12) {
            telemetry.addData("Shooting", "%.1f / 1.5 seconds", pauseTimer.seconds());
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


            /*
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
                    .build(); */

        }
    }

    private void runStateMachine() {
        switch (currentState) {
            case 0: // Start driving to the first shoot position
                follower.followPath(paths.driveToShootPose);
                currentState = 1;
                break;
            case 1: // Wait, then start shooting (spin up motor)
                if (!follower.isBusy()) {
                    launchMotor.setPower(0.98);
                    pauseTimer.reset();
                    currentState = 2;
                }
                break;
            case 2: // Pause for spin-up, then push servo to shoot
                if (pauseTimer.seconds() >= 1.0) { // Spin up for 1 second
                    rightServo.setPosition(1.0); // Push servo to shoot
                    leftServo.setPosition(1.0);
                    pauseTimer.reset(); // Reset timer for servo action
                    currentState = 3;
                }
                break;
            case 3: // Wait for servo, then clean up
                if (pauseTimer.seconds() >= 0.5) { // Wait 0.5s for servo
                    launchMotor.setPower(0);
                    rightServo.setPosition(0); // Retract servo
                    leftServo.setPosition(0);
                    currentState = 4; // Move to next action
                }
                break;
                /*
            case 4: // Drive to the first collection point (turn on intake)
                leftIntake.setPower(0.25);
                rightIntake.setPower(1);
                follower.followPath(paths.driveToCollect1);
                currentState = 5;
                break;
            case 5: // Wait, then drive back to shoot (turn off intake)
                if (!follower.isBusy()) {
                    leftIntake.setPower(0);
                    rightIntake.setPower(0);
                    follower.followPath(paths.collect1ToShoot);
                    currentState = 6;
                }
                break;
            case 6: // Wait, then shoot (spin up motor)
                if (!follower.isBusy()) {
                    launchMotor.setPower(0.8);
                    pauseTimer.reset();
                    currentState = 7;
                }
                break;
            case 7: // Pause for spin-up, then shoot
                if (pauseTimer.seconds() >= 1.0) {
                    rightServo.setPosition(1.0);
                    leftServo.setPosition(1.0);
                    pauseTimer.reset();
                    currentState = 8;
                }
                break;
            case 8: // Wait for servo, then clean up and drive
                if (pauseTimer.seconds() >= 0.5) {
                    launchMotor.setPower(0);
                    rightServo.setPosition(0);
                    leftServo.setPosition(0);

                    leftIntake.setPower(0.25);
                    rightIntake.setPower(1);
                    follower.followPath(paths.driveToCollect2);
                    currentState = 9;
                }
                break;
            case 9: // Wait, then drive back to shoot (turn off intake)
                if (!follower.isBusy()) {
                    leftIntake.setPower(0);
                    rightIntake.setPower(0);
                    follower.followPath(paths.collect2ToShoot);
                    currentState = 10;
                }
                break;
            case 10: // Wait, then shoot
                if (!follower.isBusy()) {
                    launchMotor.setPower(0.8);
                    pauseTimer.reset();
                    currentState = 11;
                }
                break;
            case 11: // Pause for spin-up, then shoot
                if (pauseTimer.seconds() >= 1.0) {
                    rightServo.setPosition(1.0);
                    leftServo.setPosition(1.0);
                    pauseTimer.reset();
                    currentState = 12;
                }
                break;
            case 12: // Wait for servo, clean up, and drive
                if (pauseTimer.seconds() >= 0.5) {
                    launchMotor.setPower(0);
                    rightServo.setPosition(0);
                    leftServo.setPosition(0);

                    leftIntake.setPower(0.25);
                    rightIntake.setPower(1);
                    follower.followPath(paths.driveToCollect3);
                    currentState = 13;
                }
                break;
            case 13: // Wait, then drive back to shoot (turn off intake)
                if (!follower.isBusy()) {
                    leftIntake.setPower(0);
                    rightIntake.setPower(0);
                    follower.followPath(paths.collect3ToShoot);
                    currentState = 14;
                }
                break;
            case 14: // Wait, then final shoot
                if (!follower.isBusy()) {
                    launchMotor.setPower(0.8);
                    pauseTimer.reset();
                    currentState = 15;
                }
                break;  */
            case 15: // Pause for spin-up, then final shoot
                if(pauseTimer.seconds() >= 1.0) {
                    rightServo.setPosition(1.0);
                    leftServo.setPosition(1.0);
                    pauseTimer.reset();
                    currentState = 16;
                }
                break;
            case 16: // Wait for servo, then turn off motor
                if(pauseTimer.seconds() >= 0.5) {
                    launchMotor.setPower(0);
                    rightServo.setPosition(0);
                    leftServo.setPosition(0);
                    currentState = 17;
                }
                break;
            case 17: // Autonomous is complete
                break;
        }
    }
}
