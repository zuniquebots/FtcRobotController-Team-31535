package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants; // Replace with your actual Constants file

@Autonomous(name = "BlueNearShotAuto1", group = "PedroPathing")
public class FarShotAuto extends LinearOpMode {

    // --- Pathing Declarations ---
    private final Pose START_POSE = new Pose(0, 96, Math.toRadians(0));
    private final Pose FAR_SHOT_POSE = new Pose(-120, 100, Math.toRadians(75));
    private final Pose outside = new Pose(-55,100,Math.toRadians(0));


    // --- Hardware Declarations for Other Motors ---
    private DcMotor launchMotor;
    private DcMotor leftIntake;
    private DcMotor rightIntake;
    private Servo leftServo;
    private Servo rightServo;
    private ColorSensor colorSensor;

    // --- Helper class for Color Conversion and Detection ---
    public static class ColorUtil {
        // Define HSV color ranges for your target colors
        // These values will need to be tuned for your specific sensor and lighting conditions
        // H is 0-360, S and V are 0-1
        private static final float[] GREEN_HSV_MIN = {100, 0.4f, 0.2f};
        private static final float[] GREEN_HSV_MAX = {140, 1.0f, 1.0f};
        private static final float[] PURPLE_HSV_MIN = {250, 0.4f, 0.2f};
        private static final float[] PURPLE_HSV_MAX = {290, 1.0f, 1.0f};

        public static boolean isGreenOrPurple(int r, int g, int b) {
            float[] hsv = new float[3];
            Color.RGBToHSV(r, g, b, hsv);

            if (hsv[0] >= GREEN_HSV_MIN[0] && hsv[0] <= GREEN_HSV_MAX[0] &&
                    hsv[1] >= GREEN_HSV_MIN[1] && hsv[1] <= GREEN_HSV_MAX[1] &&
                    hsv[2] >= GREEN_HSV_MIN[2] && hsv[2] <= GREEN_HSV_MAX[2]) {
                return true;
            }

            if (hsv[0] >= PURPLE_HSV_MIN[0] && hsv[0] <= PURPLE_HSV_MAX[0] &&
                    hsv[1] >= PURPLE_HSV_MIN[1] && hsv[1] <= PURPLE_HSV_MAX[1] &&
                    hsv[2] >= PURPLE_HSV_MIN[2] && hsv[2] <= PURPLE_HSV_MAX[2]) {
                return true;
            }
            return false;
        }
    }


    @Override
    public void runOpMode() {
        // *** INITIALIZATION ***
        Follower follower = Constants.createFollower(hardwareMap);
        initializeHardware();
        follower.setStartingPose(START_POSE);

        telemetry.addData("Status", "Initialized. Start Pose: " + START_POSE);
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        // *** PATH 1: DRIVE TO FAR SHOT POSE ***
        try {
            PathChain farShotPath = follower.pathBuilder()
                    .addPath(new BezierLine(START_POSE, FAR_SHOT_POSE))
                    .setLinearHeadingInterpolation(START_POSE.getHeading(), FAR_SHOT_POSE.getHeading())
                    .build();
            follower.followPath(farShotPath);
            waitForPath(follower); // Wait for the path to complete
        } catch (Exception e) {
            handlePathError(e);
        }

        // *** ACTIONS 1: SHOOT FROM FAR SHOT POSE ***
        telemetry.addData("Status", "Arrived at destination. Firing launcher!");
        telemetry.update();
        runShooterSequence();

        // *** PATH 2: DRIVE TO THE 'OUTSIDE' POSE ***
        try {
            telemetry.addData("Status", "Driving to the outside position...");
            telemetry.update();
            PathChain outsidePath = follower.pathBuilder()
                    // Start the new path from the robot's current position
                    .addPath(new BezierLine(follower.getPose(), outside))
                    .setLinearHeadingInterpolation(follower.getPose().getHeading(), outside.getHeading())
                    .build();
            follower.followPath(outsidePath);
            waitForPath(follower); // Wait for this second path to complete
        } catch (Exception e) {
            handlePathError(e);
        }

        telemetry.addData("Status", "Autonomous Routine Complete!");
        telemetry.update();
        sleep(2000); // Pause at the end
    }

    /**
     * Initializes all hardware components.
     */
    private void initializeHardware() {
        launchMotor = hardwareMap.get(DcMotor.class, "launchMotor");
        leftIntake = hardwareMap.get(DcMotor.class, "leftIntakeMotor");
        rightIntake = hardwareMap.get(DcMotor.class, "rightIntakeMotor");
        leftServo = hardwareMap.get(Servo.class, "leftServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        launchMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        rightIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        leftServo.setDirection(Servo.Direction.FORWARD);
        rightServo.setDirection(Servo.Direction.REVERSE);

        launchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Helper method to wait for a path to finish while updating the follower.
     * @param follower The Follower instance to monitor.
     */
    private void waitForPath(Follower follower) {
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            telemetry.addData("Status", "Following Path...");
            telemetry.addData("Current Pose", follower.getPose().toString());
            telemetry.update();
        }
    }

    /**
     * Runs the sequence for shooting pixels.
     */
    private void runShooterSequence() {
        launchMotor.setPower(0.5);
        sleep(1000);
        fireServos(); // First shot
        leftIntake.setPower(0.15);
        rightIntake.setPower(0.35);
        launchMotor.setPower(0.4);
        sleep(1500); // Feed second pixel
        fireServos(); // Second shot
        // Reset all motors
        launchMotor.setPower(0);
        leftIntake.setPower(0);
        rightIntake.setPower(0);
        leftServo.setPosition(0);
        rightServo.setPosition(0);
    }

    /**
     * Helper method to fire servos.
     */
    private void fireServos() {
        leftServo.setPosition(1);
        rightServo.setPosition(1);
        sleep(1500); // Give time for servo to move
        leftServo.setPosition(0);
        rightServo.setPosition(0);
        sleep(500); // Give time to return
    }

    /**
     * Helper method to display an error and stop.
     * @param e The exception that occurred.
     */
    private void handlePathError(Exception e) {
        telemetry.addData("ERROR", "Path execution failed: " + e.getMessage());
        telemetry.update();
        sleep(5000); // Keep the error message on screen
    }
}
