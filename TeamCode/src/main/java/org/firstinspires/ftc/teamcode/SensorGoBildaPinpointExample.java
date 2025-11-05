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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;


import java.util.Locale;

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

@Autonomous(name="goBILDA Pinpoint Example", group="Linear OpMode")

public class SensorGoBildaPinpointExample extends LinearOpMode {

    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer

    double oldTime = 0;

    private DcMotor launchMotor;
    private DcMotor leftIntake;
    private DcMotor rightIntake;
    private Servo rightServo;
    private Servo leftServo;
    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;



    public void navigation(Pose2D target, double moveSpeed) {
        final double DISTANCE_TOLERANCE = 5.0; // In millimeters, how close we need to be to stop.

        // The loop is the core of the navigation. It continues as long as the opMode is active.
        while (opModeIsActive()) {
            // STEP 1: Get the robot's current position in every loop iteration.
            odo.update();
            Pose2D current = odo.getPosition();

            // STEP 2: CORRECTED - Calculate the error between the target and the current position.
            double errorX = target.getX(DistanceUnit.MM) - current.getX(DistanceUnit.MM);
            double errorY = target.getY(DistanceUnit.MM) - current.getY(DistanceUnit.MM);

            // STEP 3: Check if the robot has arrived at the target.
            if (Math.abs(errorX) <= Math.abs(DISTANCE_TOLERANCE) && Math.abs(errorY) <= Math.abs(DISTANCE_TOLERANCE)) {
                    // If we are close enough, stop all motors and exit the loop.
                leftFrontDrive.setPower(0);
                rightFrontDrive.setPower(0);
                leftBackDrive.setPower(0);
                rightBackDrive.setPower(0);
                break; // Exit the navigation method.
            }

            // STEP 4: Convert the field-centric error (errorX, errorY) to robot-centric power.
            double robotAngleRad = current.getHeading(AngleUnit.RADIANS);
            double forwardPower = errorX * Math.cos(robotAngleRad) + errorY * Math.sin(robotAngleRad);
            double strafePower  = -errorX * Math.sin(robotAngleRad) + errorY * Math.cos(robotAngleRad);

            // For this simple A to B movement, we aren't turning.
            double turnPower = 0;

            // STEP 5: Normalize the powers
            double maxPower = Math.max(1.0, Math.abs(forwardPower) + Math.abs(strafePower));
            forwardPower /= maxPower;
            strafePower /= maxPower;

            // STEP 6: Calculate the power for each of the four mecanum wheels.
            double frontLeftPower  = (forwardPower + strafePower + turnPower) * moveSpeed;
            double frontRightPower = (forwardPower - strafePower - turnPower) * moveSpeed;
            double backLeftPower   = (forwardPower - strafePower + turnPower) * moveSpeed;
            double backRightPower  = (forwardPower + strafePower - turnPower) * moveSpeed;

            // STEP 7: Set the power on the motors.
            leftFrontDrive.setPower(frontLeftPower);
            rightFrontDrive.setPower(frontRightPower);
            leftBackDrive.setPower(backLeftPower);
            rightBackDrive.setPower(backRightPower);

            // STEP 8: Provide telemetry for debugging.
            telemetry.addData("Target", String.format(Locale.US, "X: %.1f, Y: %.1f", target.getX(DistanceUnit.MM), target.getY(DistanceUnit.MM)));
            telemetry.addData("Current", String.format(Locale.US, "X: %.1f, Y: %.1f, H: %.1f", current.getX(DistanceUnit.MM), current.getY(DistanceUnit.MM), current.getHeading(AngleUnit.DEGREES)));
            telemetry.addData("Error", String.format(Locale.US, "eX: %.1f, eY: %.1f", errorX, errorY));
            telemetry.update();
        }
        // Ensure motors are off if the opmode is stopped prematurely.
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }





    @Override
    public void runOpMode() {

        // --- INITIALIZATION ---
        // (Your existing initialization code is perfect and does not need to be changed)
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "PinpointComputer");
        launchMotor = hardwareMap.get(DcMotor.class, "launchMotor");
        leftIntake = hardwareMap.get(DcMotor.class, "leftIntakeMotor");
        rightIntake = hardwareMap.get(DcMotor.class, "rightIntakeMotor");
        rightServo = hardwareMap.get(Servo.class, "rightServo");
        leftServo = hardwareMap.get(Servo.class, "leftServo");
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");

        // Motor and Servo Directions
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
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
        odo.setOffsets(-84.0, -168.0, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();

        telemetry.addLine("Initialization Complete. Ready to start.");
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        resetRuntime();

        // --- AUTONOMOUS SEQUENCE ---
        // This sequence runs only ONCE after you press start.
        // --- AUTONOMOUS SEQUENCE ---
// This sequence runs only ONCE after you press start.
        if (opModeIsActive()) {

            // --- GO TO TARGET 1 ---
            telemetry.addLine("Driving to (X:0, Y:200)...");
            telemetry.update();

            // CORRECTED: Create a target pose to drive to X=0, Y=200, with a heading of 0 degrees.
// CORRECTED: Create a target pose to drive to X=0, Y=200, with a heading of 0 degrees.
            Pose2D target1 = new Pose2D(DistanceUnit.MM,0, 20, AngleUnit.RADIANS,Math.toRadians(0));

            // Call the navigation method to drive there at 25% speed.
            navigation(target1, 0.25);

            telemetry.addLine("Arrived at Target 1.");
            telemetry.update();
            sleep(1000); // Pause for 1 second

        }


        // The OpMode will automatically stop after the sequence above is complete.
    }

}
