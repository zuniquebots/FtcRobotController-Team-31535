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

import org.firstinspires.ftc.teamcode.pedroPathing.Constants1; // Replace with your actual Constants file

@Autonomous(name = "Red Near Shot Auto", group = "PedroPathing")
public class RedClose extends LinearOpMode {

    // --- Pathing Declarations ---
    private final Pose START_POSE = new Pose(8, 80, Math.toRadians(90));
    private final Pose FAR_SHOT_POSE = new Pose(22, 80, Math.toRadians(70));
    // The control point is now the final destination
    private final Pose CONTROL_DESTINATION = new Pose(35,120,Math.toRadians(0));//-195


    // --- Hardware Declarations for Other Motors ---
    private DcMotor launchMotor;
    private DcMotor leftIntake;
    private DcMotor rightIntake;
    private Servo leftServo;
    private Servo rightServo;
    private ColorSensor colorSensor;

    // --- Helper class for Color Conversion and Detection ---
    public static class ColorUtil {
        private static final float[] GREEN_HSV_MIN = {100, 0.4f, 0.2f};
        private static final float[] GREEN_HSV_MAX = {140, 1.0f, 1.0f};
        private static final float[] PURPLE_HSV_MIN = {250, 0.4f, 0.2f};
        private static final float[] PURPLE_HSV_MAX = {290, 1.0f, 1.0f};

        public static boolean isGreenOrPurple(int r, int g, int b) {
            float[] hsv = new float[3];
            Color.RGBToHSV(r, g, b, hsv);
            if (hsv[0] >= GREEN_HSV_MIN[0] && hsv[0] <= GREEN_HSV_MAX[0] && hsv[1] >= GREEN_HSV_MIN[1] && hsv[1] <= GREEN_HSV_MAX[1] && hsv[2] >= GREEN_HSV_MIN[2] && hsv[2] <= GREEN_HSV_MAX[2]) return true;
            if (hsv[0] >= PURPLE_HSV_MIN[0] && hsv[0] <= PURPLE_HSV_MAX[0] && hsv[1] >= PURPLE_HSV_MIN[1] && hsv[1] <= PURPLE_HSV_MAX[1] && hsv[2] >= PURPLE_HSV_MIN[2] && hsv[2] <= PURPLE_HSV_MAX[2]) return true;
            return false;
        }
    }


    @Override
    public void runOpMode() {

        // *** INITIALIZATION ***
        Follower follower = Constants1.createFollower(hardwareMap);
        launchMotor = hardwareMap.get(DcMotor.class, "launchMotor");

        leftIntake = hardwareMap.get(DcMotor.class, "leftIntakeMotor");
        rightIntake = hardwareMap.get(DcMotor.class, "rightIntakeMotor");

        leftServo  = hardwareMap.get(Servo.class, "leftServo");
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
            waitForPath(follower);
        } catch (Exception e) {
            handlePathError(e);
        }

        // *** ACTIONS 1: SHOOT FROM FAR SHOT POSE ***
        telemetry.addData("Status", "Arrived at far shot. Firing launcher!");
        telemetry.update();
        launchMotor.setPower(0.575);
        sleep(1000);
        fireServos();
        launchMotor.setPower(0.4);
        leftIntake.setPower(0.15);
        rightIntake.setPower(0.35);
        sleep(2000);
        fireServos();
        launchMotor.setPower(0);
        leftIntake.setPower(0);
        rightIntake.setPower(0);

        // *** PATH 2: DRIVE TO THE FINAL (CONTROL) DESTINATION ***
        try {
            telemetry.addData("Status", "Driving to final point...");
            telemetry.update();
            PathChain intermediatePath = follower.pathBuilder()
                    .addPath(new BezierLine(follower.getPose(), CONTROL_DESTINATION))
                    .setLinearHeadingInterpolation(follower.getPose().getHeading(), CONTROL_DESTINATION.getHeading())
                    .build();
            follower.followPath(intermediatePath);
            waitForPath(follower);
        } catch (Exception e) {
            handlePathError(e);
        }

        telemetry.addData("Status", "Autonomous Routine Complete!");
        telemetry.update();
        sleep(2000);
    }

    /**
     * Helper method to wait for a path to finish.
     * @param follower The Follower instance to monitor.
     */
    private void waitForPath(Follower follower) {
        while(opModeIsActive() && follower.isBusy()) {
            follower.update();
            telemetry.addData("Status", "Following Path...");
            telemetry.addData("Current Pose", follower.getPose().toString());
            telemetry.update();
        }
        // Stop motors after path completion to ensure no coasting
        launchMotor.setPower(0);
        leftIntake.setPower(0);
        rightIntake.setPower(0);
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
