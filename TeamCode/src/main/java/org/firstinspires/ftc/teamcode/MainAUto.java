/*   MIT License
 *   Copyright (c) [2025] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/*
This opmode shows how to use the goBILDA® Pinpoint Odometry Computer.
The goBILDA Odometry Computer is a device designed to solve the Pose Exponential calculation
commonly associated with Dead Wheel Odometry systems. It reads two encoders, and an integrated
system of senors to determine the robot's current heading, X position, and Y position.

it uses an ESP32-S3 as a main cpu, with an STM LSM6DSV16X IMU.
It is validated with goBILDA "Dead Wheel" Odometry pods, but should be compatible with any
quadrature rotary encoder. The ESP32 PCNT peripheral is speced to decode quadrature encoder signals
at a maximum of 40mhz per channel. Though the maximum in-application tested number is 130khz.

The device expects two perpendicularly mounted Dead Wheel pods. The encoder pulses are translated
into mm and their readings are transformed by an "offset", this offset describes how far away
the pods are from the "tracking point", usually the center of rotation of the robot.

Dead Wheel pods should both increase in count when moved forwards and to the left.
The gyro will report an increase in heading when rotated counterclockwise.
tt
The Pose Exponential algorithm used is described on pg 181 of this book:
https://github.com/calcmogul/controls-engineering-in-frc

For support, contact tech@gobilda.com

-Ethan Doak
 */

@Autonomous(name="Main Auto", group="Linear OpMode")

public class MainAUto extends LinearOpMode {

    GoBildaPinpointDriver pinpointDriver; // Declare OpMode member for the Odometry Computer
    private DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive; // Assuming a 4-wheel drive
    private DcMotor launchMotor, leftIntake, rightIntake;
    private Servo rightServo, leftServo;
    private ColorSensor colorSensor;
    private final double DISTANCE_TOLERANCE = 5.0; // In millimeters, how close we need to be to stop.

    @Override
    public void runOpMode() {
        AngleUnit angleUnit = AngleUnit.DEGREES;
        DistanceUnit distanceUnit = DistanceUnit.MM;

        //STEP 1: Initialize Hardware Settings
        initialize();

        Pose2D current = new Pose2D(distanceUnit, 0, 0, angleUnit, Math.toRadians(0));  // Set the robot's initial position  - that converts an angle of 0 degrees to its equivalent value in radians.
        pinpointDriver.setPosition(current);
        pinpointDriver.recalibrateIMU();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();         // Wait for the game to start (driver presses START)
        resetRuntime();

        if (opModeIsActive()) {
            leftIntake.setPower(-0.35);
            rightIntake.setPower(-1);

            // STEP 2: Set the Target
            Pose2D target = new Pose2D(distanceUnit, 0, -400, angleUnit, Math.toRadians(0));  // Set the robot's target position
            telemetry.addData("Status", "Moving to Target");
            telemetry.addData("Current X, Y", "%.2f, %.2f", current.getX(distanceUnit), current.getY(distanceUnit));
            telemetry.addData("Target X, Y", "%.2f, %.2f", target.getX(distanceUnit), target.getY(distanceUnit));
            telemetry.update();

            //STEP 3: Move to Target
            moveToTarget(target.getX(distanceUnit), target.getY(distanceUnit), 0, distanceUnit, angleUnit);
            telemetry.addData("Status", "Reached Target");
            telemetry.update();
            //Step 4: Check whether there are balls in the robot
            if (opModeIsActive()) {
                // ... (your moveToTarget2 call)

                telemetry.addData("Status", "Reached Target");
                telemetry.update();

                // --- CORRECTED CODE FOR STEP 4 ---
                //Step 4: Check which pixel is detected

                // Get the current RGB values from the sensor
                int redValue = colorSensor.red();
                int greenValue = colorSensor.green();
                int blueValue = colorSensor.blue();

                float[] hsvValues = new float[3];
                android.graphics.Color.RGBToHSV(redValue, greenValue, blueValue, hsvValues);


                if (isGreenOrPurple(hsvValues)) {
                    telemetry.addLine("Pixel Detected: Green or Purple");
                    // Add actions here, e.g., run launcher
                    launchMotor.setPower(0.4);
                    sleep(1000);
                    leftIntake.setPower(0);

                } else {
                    telemetry.addLine("No Green or Purple Pixel Detected");
                }

                telemetry.addData("Hue", "%.1f", hsvValues[0]);
                telemetry.addData("Saturation", "%.1f", hsvValues[1]);
                telemetry.addData("Value", "%.1f", hsvValues[2]);
                telemetry.addData("Status", "Autonomous Finished");
                telemetry.update();
                sleep(1000);
            }


        }


        // The OpMode will automatically stop after the sequence above is complete.
    }

    private void initialize() {
        // --- INITIALIZATION ---
        // (Your existing initialization code is perfect and does not need to be changed)
        pinpointDriver = hardwareMap.get(GoBildaPinpointDriver.class, "PinpointComputer");

        launchMotor = hardwareMap.get(DcMotor.class, "launchMotor");

        leftIntake = hardwareMap.get(DcMotor.class, "leftIntakeMotor");
        rightIntake = hardwareMap.get(DcMotor.class, "rightIntakeMotor");

        rightServo = hardwareMap.get(Servo.class, "rightServo");
        leftServo = hardwareMap.get(Servo.class, "leftServo");

        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");


        // Motor and Servo Directions
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);   // TODO: SET REVERSE
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);

        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);  // TODO: SET FORWARD
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        launchMotor.setDirection(DcMotor.Direction.FORWARD);
        rightIntake.setDirection(DcMotor.Direction.REVERSE);
        leftIntake.setDirection(DcMotor.Direction.REVERSE);
        rightServo.setDirection(Servo.Direction.REVERSE);
        leftServo.setDirection(Servo.Direction.FORWARD);


        // Set Zero Power Behavior
        leftIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set motor modes
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Pinpoint Configuration
        pinpointDriver.setOffsets(-84.0, -168.0, DistanceUnit.MM);
        pinpointDriver.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpointDriver.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinpointDriver.resetPosAndIMU();


    }

    public void moveToTarget(double targetX, double targetY, double targetHeading, DistanceUnit distanceUnit, AngleUnit angleUnit) {
        double currentX = 0.0;
        double currentY = 0.0;
        double currentHeading = 0.0;

        while (opModeIsActive() && !isAtTarget(targetX, targetY, targetHeading, distanceUnit, angleUnit)) {
            pinpointDriver.update();
            // Get current position
            Pose2D currentPose = pinpointDriver.getPosition();
            currentX = currentPose.getX(distanceUnit);
            currentY = currentPose.getY(distanceUnit);
            currentHeading = currentPose.getHeading(angleUnit);  // Angle to target relative to robot's current heading

            // Calculate error (distance to target)  & Calculate movement direction (angle towards target)
            double errorX = targetX - currentX;
            double errorY = targetY - currentY;
            double distance = Math.sqrt(Math.pow(errorX, 2) + Math.pow(errorY, 2));
            double angleToTarget = Math.atan2(errorY, errorX);

            // Normalize the angle to be between -PI and PI
            double relativeAngle = angleToTarget - currentHeading;
            relativeAngle = AngleUnit.RADIANS.normalize(relativeAngle);

            // --- PID Logic (PLACEHOLDER) ---
            // Implement a PID (Proportional-Integral-Derivative) controller to calculate the motor power needed to reduce the error.
            double drivePower = distance * 0.003; // Adjust this P constant
            double driveXPower = calculatePIDOutputX(errorX);
            double driveYPower = calculatePIDOutputY(errorY);
            double driveTurnPower = calculatePIDOutputHeading(relativeAngle);

            // Apply powers to motors (assuming Mecanum drive)
            // Need a function to convert driveX, driveY, driveTurn into individual motor powers
            moveRobot(driveXPower, driveYPower, driveTurnPower);


            // Add telemetry for debugging
            telemetry.addData("Status", "Running");
            telemetry.addData("Target X, Y", "%.2f, %.2f,%.2f", targetX, targetY, targetHeading);
            telemetry.addData("Current X, Y", "%.2f, %.2f,%.2f", currentX, currentY, currentHeading);
            telemetry.addData("Distance Remaining", "%.2f", distance);
            telemetry.update();

            // Allow the OpMode to react to new commands or stop
            idle();

        }

        telemetry.addData("Status", "Arrived at Target.");
        telemetry.addData("Target X, Y", "%.2f, %.2f", targetX, targetY);
        telemetry.addData("Current X, Y", "%.2f, %.2f", currentX, currentY);
        telemetry.update();

        sleep(1000); // Pause for 1 second

        // Stop motors once target is reached
        stopMotors();
        telemetry.addData("Status", "Stopping Motors.");
        telemetry.update();
        sleep(200); // Pause for 0.2 second

    }

    private boolean isAtTarget(double targetX, double targetY, double targetHeading, DistanceUnit distanceUnit, AngleUnit angleUnit) {
        // Define tolerance levels for position and heading
        double positionTolerance = 5.0; // e.g., 20mm
        double headingTolerance = 2.0;   // e.g., 2 degrees

        double currentX = pinpointDriver.getPosX(distanceUnit);
        double currentY = pinpointDriver.getPosY(distanceUnit);
        double currentHeading = pinpointDriver.getHeading(angleUnit);

        double errorX = Math.abs(targetX - currentX);
        double errorY = Math.abs(targetY - currentY);
        double errorHeading = Math.abs(targetHeading - currentHeading);

        return (errorX < positionTolerance && errorY < positionTolerance && errorHeading < headingTolerance);
    }

    // Placeholder functions for your PID logic and motor control
    private double calculatePIDOutputX(double errorX) {
        // Implement your X-axis PID controller here
        // Return a motor power value (e.g., between -1.0 and 1.0)
        // A simple proportional control:
        double Kp = 0.01; // Adjust Kp as needed
        return Range.clip(errorX * Kp, -0.5, 0.5);
    }

    private double calculatePIDOutputY(double errorY) {
        // Implement your Y-axis PID controller here
        double Kp = 0.01;
        return Range.clip(errorY * Kp, -0.5, 0.5);
    }

    private double calculatePIDOutputHeading(double errorHeading) {
        // Implement your heading PID controller here
        double Kp = 0.01;
        return Range.clip(errorHeading * Kp, -0.5, 0.5);
    }

    private void moveRobot(double driveX, double driveY, double driveTurn) {
        // Implement motor power application (Mecanum drive formula)
        double frontLeftPower = driveX + driveY + driveTurn;
        double frontRightPower = driveX - driveY - driveTurn;
        double backLeftPower = driveX - driveY + driveTurn;
        double backRightPower = driveX + driveY - driveTurn;

        // Clip the power values to be within the valid range [-1.0, 1.0]
        frontLeftPower = Range.clip(frontLeftPower, -1.0, 1.0);
        frontRightPower = Range.clip(frontRightPower, -1.0, 1.0);
        backLeftPower = Range.clip(backLeftPower, -1.0, 1.0);
        backRightPower = Range.clip(backRightPower, -1.0, 1.0);

        leftFrontDrive.setPower(0.8 * -frontLeftPower);
        rightFrontDrive.setPower(0.8 * -frontRightPower);

        leftBackDrive.setPower(0.8 * -backLeftPower);
        rightBackDrive.setPower(0.8 * -backRightPower);
    }

    private void stopMotors() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    /**
     * Checks if the given HSV values correspond to either green or purple.
     * Using HSV is more reliable across different lighting conditions than RGB.
     *
     * @param hsv An array of floats containing Hue, Saturation, and Value.
     * @return True if the color is confidently detected as green or purple, false otherwise.
     */
    public static boolean isGreenOrPurple(float[] hsv) {
        // hsv[0] is Hue (0-360), hsv[1] is Saturation (0-1), hsv[2] is Value (0-1)
        float hue = hsv[0];
        float saturation = hsv[1];
        float value = hsv[2];

        // Define hue ranges for colors. These may need slight tuning.
        // Green hues are typically between 80 and 150.
        boolean isGreen = (hue >= 150 && hue <= 163);
        // Purple hues are typically between 260 and 310.
        boolean isPurple = (hue >= 165 && hue <= 240);

        // Add saturation and value checks to avoid false positives on whites/greys/blacks.
        // The color should be saturated enough and not too dark.
        boolean isAColor = saturation > 0.45 && value > 0.18;

        return (isGreen || isPurple) && isAColor;
    }

// The old ColorSensor(int, int, int) method with the while loop has been removed
// as its logic is now correctly handled in runOpMode.
}






