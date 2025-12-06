package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor; // Import VoltageSensor
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Red Auto Pathing Close", group = "Pedro Pathing")
public class PedroPathingStarter_RedClose extends OpMode {

    private Follower follower;
    private int currentState;
    private AutonomousPaths paths;
    private ElapsedTime pauseTimer = new ElapsedTime();
    private ElapsedTime matchTimer = new ElapsedTime(); // New timer for total match time

    // Hardware
    private DcMotor launchMotor;
    private DcMotor leftIntake;
    private DcMotor rightIntake;
    private Servo rightServo; // Added servo for shooting
    private Servo leftServo;  // Added servo for shooting
    private Servo directionServo; // Added direction servo
    private VoltageSensor voltageSensor; // Added VoltageSensor


    // ------------ CORRECTED POSES ------------
    // START_POSE and SHOOT_POSE have been adjusted
    public static final Pose START_POSE = new Pose(82, 135, Math.toRadians(90));
    public static final Pose SHOOT_POSE = new Pose(119, 120, Math.toRadians(135));
    // New shooting pose for subsequent shots
    public static final Pose Shoot_Pose_1 = new Pose(119, 120, Math.toRadians(-135));


    // Staging poses for collection
    public static final Pose COLLECT_POSE1 = new Pose(98, 35, Math.toRadians(180));
    public static final Pose COLLECT_POSE2 = new Pose(98, 60, Math.toRadians(180));
    public static final Pose COLLECT_POSE3 = new Pose(98, 85, Math.toRadians(180));
    // Actual collection poses
    public static final Pose Actual_Collect3= new Pose(118, 85, Math.toRadians(180));
    public static final Pose Actual_Collect2= new Pose(118, 60, Math.toRadians(180));
    public static final Pose Actual_Collect1= new Pose(118, 35, Math.toRadians(180));

    // Parking pose, corrected typo
    public static final Pose Outside_Pose = new Pose(120, 64, Math.toRadians(180));

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(START_POSE);
        follower.setMaxPower(0.9);

        // --- Initialize Hardware ---
        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub"); // Initialize sensor
        launchMotor = hardwareMap.get(DcMotor.class, "launchMotor");
        leftIntake = hardwareMap.get(DcMotor.class, "leftIntakeMotor");
        rightIntake = hardwareMap.get(DcMotor.class, "rightIntakeMotor");
        rightServo = hardwareMap.get(Servo.class, "rightServo");
        leftServo = hardwareMap.get(Servo.class, "leftServo");
        directionServo =  hardwareMap.get(Servo.class, "tiltServo");


        // --- Set Motor and Servo Directions ---
        launchMotor.setDirection(DcMotor.Direction.FORWARD);
        rightIntake.setDirection(DcMotor.Direction.REVERSE);
        leftIntake.setDirection(DcMotor.Direction.FORWARD);
        rightServo.setDirection(Servo.Direction.REVERSE);
        leftServo.setDirection(Servo.Direction.FORWARD);
        directionServo.setDirection(Servo.Direction.REVERSE);


        paths = new AutonomousPaths(follower);
        currentState = 0;
        // Set servos to initial position (holding the ball)
        rightServo.setPosition(0);
        leftServo.setPosition(0);
        directionServo.setPosition(0); // Set direction servo to initial position

        // --- WARM UP LAUNCHER ---
        telemetry.addData("Status", "Initialization Complete. Launcher Warming Up.");
        telemetry.update();
    }

    @Override
    public void init_loop() {}


    @Override
    public void start() {
        pauseTimer.reset();
        // --- START INTAKE AT FULL POWER ---
        leftIntake.setPower(0);
        rightIntake.setPower(0);
        launchMotor.setPower(getVoltageCompensatedPower());
        directionServo.setPosition(0.04);
        matchTimer.reset(); // Start the master match timer
        telemetry.addData("Status", "Autonomous Started");
        telemetry.addData("Initial Launcher Power", "%.3f at %.2fV", getVoltageCompensatedPower(), voltageSensor.getVoltage());
        telemetry.update();

    }

    /**
     * Calculates the appropriate launch motor power based on the current battery voltage.
     * Uses a linear equation derived from two known data points.
     * Point 1: 14.0V -> 0.4 power
     * Point 2: 13.5V -> 0.425 power
     *
     * @return The calculated motor power, clipped between 0.0 and 1.0.
     */
    private double getVoltageCompensatedPower() {
        double currentVoltage = voltageSensor.getVoltage();

        // Define our data points
        double voltage1 = 14.0;
        double power1 = 0.4;
        double voltage2 = 13.5;
        double power2 = 0.42;

        // Avoid division by zero, though it's fixed with these constants
        if (voltage1 == voltage2) return power1;

        // Calculate the slope (m) of the linear equation y = mx + c
        double slope = (power2 - power1) / (voltage2 - voltage1);

        // Calculate the y-intercept (c)
        double intercept = power1 - (slope * voltage1);

        // Calculate the target power using the linear equation
        double calculatedPower = (slope * currentVoltage) + intercept;

        // Clip the power to a safe range [0, 1] to avoid errors
        return Range.clip(calculatedPower, 0.0, 1.0);
    }

    @Override
    public void loop() {
        follower.update();
        launchMotor.setPower(getVoltageCompensatedPower());
        // Master timer check for parking is kept, in case the first move takes too long
        if (matchTimer.seconds() > 26.5 && currentState != 100 && currentState != 101) {
            follower.setMaxPower(1);
            // Stop all motors immediately
            launchMotor.setPower(0);
            leftIntake.setPower(0);
            rightIntake.setPower(0);

            // Create a new path from the current pose to the parking pose
            PathChain parkPath = follower.pathBuilder()
                    .addPath(new BezierLine(follower.getPose(), Outside_Pose))
                    .setLinearHeadingInterpolation(follower.getPose().getHeading(), Outside_Pose.getHeading())
                    .build();
            follower.followPath(parkPath);
            currentState = 100; // Go to a dedicated "parking" state
        }


        runStateMachine();

        telemetry.addData("Current State", currentState);
        telemetry.addData("Match Time", "%.1f / 30.0", matchTimer.seconds());
        telemetry.addData("Robot X", follower.getPose().getX());
        telemetry.addData("Robot Y", follower.getPose().getY());
        telemetry.addData("Robot Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Battery Voltage", "%.2f V", voltageSensor.getVoltage());
        telemetry.addData("Compensated Power", "%.3f", getVoltageCompensatedPower());

        if (currentState == 2) {
            telemetry.addData("Shooting", "%.1f / 2.0 seconds", pauseTimer.seconds());
        }
        telemetry.update();
    }

    public static class AutonomousPaths {
        public PathChain driveToShootPose;
        // Cycle 3 Paths
        public PathChain driveToStagingCollect3;
        public PathChain driveToActualCollect3;
        public PathChain actualCollect3ToShoot;
        // Cycle 2 Paths
        public PathChain driveToStagingCollect2;
        public PathChain driveToActualCollect2;
        public PathChain actualCollect2ToShoot;
        // Cycle 1 Paths
        public PathChain driveToStagingCollect1;
        public PathChain driveToActualCollect1;
        public PathChain actualCollect1ToShoot;

        public AutonomousPaths(Follower follower) {
            // Path from Start to initial Shoot Pose
            driveToShootPose = follower.pathBuilder().addPath(new BezierLine(START_POSE, SHOOT_POSE)).setLinearHeadingInterpolation(START_POSE.getHeading(), SHOOT_POSE.getHeading()).build();

            // Paths for Collection Cycle 3
            driveToStagingCollect3 = follower.pathBuilder().addPath(new BezierLine(SHOOT_POSE, COLLECT_POSE3)).setLinearHeadingInterpolation(SHOOT_POSE.getHeading(), COLLECT_POSE3.getHeading()).build();
            driveToActualCollect3 = follower.pathBuilder().addPath(new BezierLine(COLLECT_POSE3, Actual_Collect3)).setLinearHeadingInterpolation(COLLECT_POSE3.getHeading(), Actual_Collect3.getHeading()).build();
            actualCollect3ToShoot = follower.pathBuilder().addPath(new BezierLine(Actual_Collect3, Shoot_Pose_1)).setLinearHeadingInterpolation(Actual_Collect3.getHeading(), Shoot_Pose_1.getHeading()).build();

            // Paths for Collection Cycle 2
            driveToStagingCollect2 = follower.pathBuilder().addPath(new BezierLine(SHOOT_POSE, COLLECT_POSE2)).setLinearHeadingInterpolation(SHOOT_POSE.getHeading(), COLLECT_POSE2.getHeading()).build();
            driveToActualCollect2 = follower.pathBuilder().addPath(new BezierLine(COLLECT_POSE2, Actual_Collect2)).setLinearHeadingInterpolation(COLLECT_POSE2.getHeading(), Actual_Collect2.getHeading()).build();
            actualCollect2ToShoot = follower.pathBuilder().addPath(new BezierLine(Actual_Collect2, Shoot_Pose_1)).setLinearHeadingInterpolation(Actual_Collect2.getHeading(), Shoot_Pose_1.getHeading()).build();

            // Paths for Collection Cycle 1
            driveToStagingCollect1 = follower.pathBuilder().addPath(new BezierLine(SHOOT_POSE, COLLECT_POSE1)).setLinearHeadingInterpolation(SHOOT_POSE.getHeading(), COLLECT_POSE1.getHeading()).build();
            driveToActualCollect1 = follower.pathBuilder().addPath(new BezierLine(COLLECT_POSE1, Actual_Collect1)).setLinearHeadingInterpolation(COLLECT_POSE1.getHeading(), Actual_Collect1.getHeading()).build();
            actualCollect1ToShoot = follower.pathBuilder().addPath(new BezierLine(Actual_Collect1, Shoot_Pose_1)).setLinearHeadingInterpolation(Actual_Collect1.getHeading(), Shoot_Pose_1.getHeading()).build();
        }
    }

    private void runStateMachine() {
        switch (currentState) {
            //<editor-fold desc="First Shot (Preloads)">
            case 0: // Start driving to the first shoot position
                follower.followPath(paths.driveToShootPose);
                currentState = 1;
                break;
            case 1: // Wait for path to complete, then start shooting timer
                if (!follower.isBusy()) {
                    pauseTimer.reset(); // Reset timer AFTER path is done
                    currentState = 2;
                }
                break;
            case 2:
                // Wait for spin-up, then shoot FIRST ball
                if (pauseTimer.seconds() >= 1.75) { // Wait for spin-up
                    rightServo.setPosition(1.0); // Push servo to shoot
                    leftServo.setPosition(1.0);
                    pauseTimer.reset(); // Reset timer for servo action
                    currentState = 3;
                }
                break;
            case 3:
                leftIntake.setPower(0.3);
                rightIntake.setPower(1);
                // Wait for first servo push, then retract to load second ball
                if (pauseTimer.seconds() >= 0.5) { // Wait 0.5s for servo to extend
                    rightServo.setPosition(0); // Retract servo
                    leftServo.setPosition(0);
                    pauseTimer.reset(); // Reset timer for loading
                    currentState = 4;
                }
                break;
            case 4: // Wait for second ball to load, then shoot SECOND ball
                if (pauseTimer.seconds() >= 1.75) { // Wait for ball to settle
                    rightServo.setPosition(1.0); // Push servo to shoot again
                    leftServo.setPosition(1.0);
                    pauseTimer.reset(); // Reset timer for second servo action
                    currentState = 5;
                }
                break;
            case 5: // Wait for second servo push, then clean up and start first collection cycle
                if (pauseTimer.seconds() >= 1.0) {
                    rightServo.setPosition(0); // Retract servo
                    leftServo.setPosition(0);
                    leftIntake.setPower(0);
                    rightIntake.setPower(0);
                    follower.followPath(paths.driveToStagingCollect3); // Start first collection path
                    currentState = 6;
                }
                break;
            //</editor-fold>

            //<editor-fold desc="Collect 3 Cycle">
            case 6: // Arrived at Staging 3, drive to Actual 3
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.55);
                    leftIntake.setPower(0.1);
                    rightIntake.setPower(1);
                    follower.followPath(paths.driveToActualCollect3);
                    currentState = 7;
                }
                break;
            case 7: // Arrived at Actual 3, drive back to shoot
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.9);
                    leftIntake.setPower(0);
                    rightIntake.setPower(0);
                    follower.followPath(paths.actualCollect3ToShoot);
                    currentState = 8;
                }
                break;
            case 8: // Arrived at Shoot, wait then begin 2-shot sequence
                if (!follower.isBusy()) {
                    pauseTimer.reset();
                    currentState = 9;
                }
                break;
            case 9: // Wait for spin-up, then shoot FIRST ball
                if (pauseTimer.seconds() >= 1.75) {
                    rightServo.setPosition(1.0);
                    leftServo.setPosition(1.0);
                    pauseTimer.reset();
                    currentState = 10;
                }
                break;
            case 10:
                leftIntake.setPower(0.3);
                rightIntake.setPower(1);
                // Retract servo, wait for load
                if (pauseTimer.seconds() >= 0.5) {
                    rightServo.setPosition(0);
                    leftServo.setPosition(0);
                    pauseTimer.reset();
                    currentState = 11;
                }
                break;
            case 11: // Wait for load, then shoot SECOND ball
                if (pauseTimer.seconds() >= 1.0) {
                    rightServo.setPosition(1.0);
                    leftServo.setPosition(1.0);
                    pauseTimer.reset();
                    currentState = 12;
                }
                break;
            case 12: // Retract servo and drive to Staging 2
                leftIntake.setPower(0);
                rightIntake.setPower(0);
                if (pauseTimer.seconds() >= 1.75) {
                    rightServo.setPosition(0);
                    leftServo.setPosition(0);
                    follower.followPath(paths.driveToStagingCollect2);
                    currentState = 13;
                }
                break;
            //</editor-fold>

            //<editor-fold desc="Collect 2 Cycle">
            case 13: // Arrived at Staging 2, drive to Actual 2
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.55);
                    leftIntake.setPower(0.1);
                    rightIntake.setPower(1);
                    follower.followPath(paths.driveToActualCollect2);
                    currentState = 14;
                }
                break;
            case 14: // Arrived at Actual 2, drive back to shoot
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.9);
                    leftIntake.setPower(0);
                    rightIntake.setPower(0);
                    follower.followPath(paths.actualCollect2ToShoot);
                    currentState = 15;
                }
                break;
            case 15: // Arrived at Shoot, wait then begin 2-shot sequence
                if (!follower.isBusy()) {
                    pauseTimer.reset();
                    currentState = 16;
                }
                break;
            case 16:
                // Wait for spin-up, then shoot FIRST ball
                if (pauseTimer.seconds() >= 1.5) {
                    rightServo.setPosition(1.0);
                    leftServo.setPosition(1.0);
                    pauseTimer.reset();
                    currentState = 17;
                }
                break;
            case 17:
                leftIntake.setPower(0.3);
                rightIntake.setPower(1);
                // Retract servo, wait for load
                if (pauseTimer.seconds() >= 1.5) {
                    rightServo.setPosition(0);
                    leftServo.setPosition(0);
                    pauseTimer.reset();
                    currentState = 18;
                }
                break;
            case 18: // Wait for load, then shoot SECOND ball
                if (pauseTimer.seconds() >= 0.5) {
                    rightServo.setPosition(1.0);
                    leftServo.setPosition(1.0);
                    pauseTimer.reset();
                    currentState = 19;
                }
                break;
            case 19: // Retract servo and drive to Staging 1
                if (pauseTimer.seconds() >= 1.0) {
                    rightServo.setPosition(0);
                    leftServo.setPosition(0);
                    leftIntake.setPower(0);
                    rightIntake.setPower(0);
                    follower.followPath(paths.driveToStagingCollect1);
                    currentState = 20;
                }
                break;
            //</editor-fold>

            //<editor-fold desc="Collect 1 Cycle (Final)">
            case 20: // Arrived at Staging 1, drive to Actual 1
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.55);
                    leftIntake.setPower(0.1);
                    rightIntake.setPower(1);
                    follower.followPath(paths.driveToActualCollect1);
                    currentState = 21;
                }
                break;
            case 21: // Arrived at Actual 1, drive back to shoot
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.9);
                    leftIntake.setPower(0);
                    rightIntake.setPower(0);
                    follower.followPath(paths.actualCollect1ToShoot);
                    currentState = 22;
                }
                break;
            case 22: // Arrived at Shoot, wait then begin final 2-shot sequence
                if (!follower.isBusy()) {
                    pauseTimer.reset();
                    currentState = 23;
                }
                break;
            case 23: // Wait for spin-up, then shoot FIRST ball
                if (pauseTimer.seconds() >= 1.5) {
                    rightServo.setPosition(1.0);
                    leftServo.setPosition(1.0);
                    pauseTimer.reset();
                    currentState = 24;
                }
                break;
            case 24: // Retract servo, wait for load
                if (pauseTimer.seconds() >= 0.5) {
                    rightServo.setPosition(0);
                    leftServo.setPosition(0);
                    pauseTimer.reset();
                    currentState = 25;
                }
                break;
            case 25: // Wait for load, then shoot SECOND ball
                leftIntake.setPower(0.3);
                rightIntake.setPower(1);
                if (pauseTimer.seconds() >= 1.0) {
                    rightServo.setPosition(1.0);
                    leftServo.setPosition(1.0);
                    pauseTimer.reset();
                    currentState = 26;
                }
                break;
            case 26: // Retract servo and end routine
                if (pauseTimer.seconds() >= 1.0) {
                    launchMotor.setPower(0);
                    rightServo.setPosition(0);
                    leftServo.setPosition(0);
                    leftIntake.setPower(0);
                    rightIntake.setPower(0);
                    currentState = 27; // Finish
                }
                break;
            case 27: // Autonomous is complete
                break;
            //</editor-fold>

            // --- END OF TIME FAILSAFE STATES ---
            case 100: // Parking state
                if (!follower.isBusy()) {
                    currentState = 101; // Final parked state
                }
                break;
            case 101: // Robot is parked
                // Do nothing
                break;
        }
    }
}
