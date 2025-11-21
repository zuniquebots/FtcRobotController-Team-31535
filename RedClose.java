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

@Autonomous(name = "RedCloseShot", group = "Pedro Pathing")
public class RedClose extends LinearOpMode {

    // --- Pathing Declarations ---
    private final Pose START_POSE = new Pose(0, 0, Math.toRadians(0));
    private final Pose FAR_SHOT_POSE = new Pose(-120, 0, Math.toRadians(-95));
    private final Pose Pick_Up = new Pose(75,25,Math.toRadians(-270));

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

        /**
         * Converts RGB values to HSV and checks if the color is green or purple.
         * @param r Red component (0-255)
         * @param g Green component (0-255)
         * @param b Blue component (0-255)
         * @return true if the color is within the green or purple HSV range.
         */
        public static boolean isGreenOrPurple(int r, int g, int b) {
            float[] hsv = new float[3];
            Color.RGBToHSV(r, g, b, hsv);

            // Check for Green
            if (hsv[0] >= GREEN_HSV_MIN[0] && hsv[0] <= GREEN_HSV_MAX[0] &&
                    hsv[1] >= GREEN_HSV_MIN[1] && hsv[1] <= GREEN_HSV_MAX[1] &&
                    hsv[2] >= GREEN_HSV_MIN[2] && hsv[2] <= GREEN_HSV_MAX[2]) {
                return true;
            }

            // Check for Purple
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
        // *** STEP 1: INITIALIZE THE FOLLOWER AND OTHER HARDWARE ***

        // Initialize the pathing follower
        Follower follower = Constants.createFollower(hardwareMap);

        // Initialize your other motors and servos
        launchMotor = hardwareMap.get(DcMotor.class, "launchMotor");
        leftIntake = hardwareMap.get(DcMotor.class, "leftIntakeMotor");
        rightIntake = hardwareMap.get(DcMotor.class, "rightIntakeMotor");
        leftServo  = hardwareMap.get(Servo.class, "leftServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        // Set motor directions (adjust as needed)
        launchMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        rightIntake.setDirection(DcMotorSimple.Direction.FORWARD);

        leftServo.setDirection(Servo.Direction.FORWARD);
        rightServo.setDirection(Servo.Direction.REVERSE);


        // Set zero power behavior
        launchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // *** STEP 2: SET THE INITIAL ROBOT POSE ***
        follower.setStartingPose(START_POSE);

        // *** STEP 3: WAIT FOR START ***
        telemetry.addData("Status", "Initialized. Start Pose: " + START_POSE);
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

            // Loop while the path is running, and control other motors simultaneously
            while (opModeIsActive() && follower.isBusy()) {
                // REQUIRED: Update the follower to keep the robot moving
                follower.update();
/*
                // Get RGB values from the color sensor
                int red = colorSensor.red();
                int green = colorSensor.green();
                int blue = colorSensor.blue();

                // Check if a green or purple pixel is detected
                if (ColorUtil.isGreenOrPurple(red, green, blue)) {
                    telemetry.addData("Pixel Detected", true);
                    // Slow down intake when a pixel is detected
                    leftIntake.setPower(0.1);
                    rightIntake.setPower(0.25);
                    sleep(1500);
                } else {
                    telemetry.addData("Pixel Detected", false);
                    // Run intake at full speed to search for a pixel
                    leftIntake.setPower(0.35);
                    rightIntake.setPower(1);
                    sleep(1500);
                }
                */

                // Telemetry for debugging
                telemetry.addData("Status", "Following Path & Running Intake...");
                telemetry.addData("Current Pose", follower.getPose().toString());
                //telemetry.addData("RGB", "%d, %d, %d", red, green, blue);
                telemetry.update();
            }

            // Stop the intake now that we've arrived
            leftIntake.setPower(0);
            rightIntake.setPower(0);


        } catch (Exception e) {
            telemetry.addData("ERROR", "Path execution failed: " + e.getMessage());
            telemetry.update();
            sleep(5000); // Keep the error message on screen
        }

        // *** STEP 5: PERFORM ACTIONS AFTER THE PATH IS COMPLETE ***
        telemetry.addData("Status", "Arrived at destination. Firing launcher!");
        telemetry.update();

        // Example: Run the launcher at full power for 1 second
        launchMotor.setPower(0.55);
        sleep(1000); // Wait for 1 second
        leftServo.setPosition(1);
        rightServo.setPosition(1);
        sleep(3000);
        leftServo.setPosition(0);
        rightServo.setPosition(0);
        launchMotor.setPower(0.4);
        leftIntake.setPower(0.15);
        rightIntake.setPower(0.35);
        sleep(2000);
        leftServo.setPosition(1);
        rightServo.setPosition(1);
        leftIntake.setPower(0);
        rightIntake.setPower(0);
        sleep(5000);
        launchMotor.setPower(0); // Stop the launcher
        leftServo.setPosition(0);
        rightServo.setPosition(0);
        leftIntake.setPower(0);
        rightIntake.setPower(0.0);


        telemetry.addData("Status", "Path and Actions Complete!");
        telemetry.update();
    }
}
