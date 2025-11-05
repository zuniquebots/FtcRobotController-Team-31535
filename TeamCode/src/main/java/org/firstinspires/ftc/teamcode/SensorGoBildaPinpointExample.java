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
import com.qualcomm.robotcore.util.Range;

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
    private final double DISTANCE_TOLERANCE = 5.0; // In millimeters, how close we need to be to stop.


    private void stopRobot(){
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    public void navigation(Pose2D target, double moveSpeed) {

        Pose2D current = null;

        double targetX = 200;
        double targetY = 0;
        double targetHeading = 0;

        double currentX = 0;
        double currentY = 0;
        double currentHeading = 0;


        double deltaX = 0;
        double deltaY = 0 ;
        double deltaHeading = 0;
        double absoluteAngleToTarget = 0;
        double distanceToTarget = 0;

        double drivePower = 0;
        double turnPower = 0;

        double maxPower = 0;


        double forwardPower = 0;
        double strafePower = 0;
        double frontLeftPower = 0;
        double frontRightPower = 0;
        double backLeftPower = 0;
        double backRightPower = 0;


        // The loop is the core of the navigation. It continues as long as the opMode is active.
        while (opModeIsActive() && !(isAtTarget(targetX,targetY))) {
            // STEP 1: Get the robot's current position in every loop iteration.
            odo.update();
            current = odo.getPosition();

            targetX = target.getX(DistanceUnit.MM);
            targetY = target.getY(DistanceUnit.MM);
            targetHeading = target.getHeading(AngleUnit.RADIANS);


            currentX = current.getX(DistanceUnit.MM);
            currentY = current.getY(DistanceUnit.MM);
            currentHeading = current.getHeading(AngleUnit.RADIANS);

            //Calculate error(Delta)
            deltaX = targetX - currentX;
            deltaY = targetY - currentY;
            distanceToTarget = Math.sqrt(deltaX * deltaX +  deltaY * deltaY);
            absoluteAngleToTarget = Math.atan2(deltaY, deltaX); // Calculate desired heading to target
            deltaHeading = AngleUnit.RADIANS.normalize(absoluteAngleToTarget - currentHeading);

            // STEP 3: Check if the robot has arrived at the target.
            if (Math.abs(deltaX) <= Math.abs(DISTANCE_TOLERANCE) && Math.abs(deltaY) <= Math.abs(DISTANCE_TOLERANCE)) {
                stopRobot();  // If we are close enough, stop all motors and exit the loop
                break; // Exit the navigation method.
            }

            // Look for this Formula:
            drivePower = Range.clip(distanceToTarget / 20.0, 0.1, 0.5); // Max 0.5 power
            turnPower = Range.clip(deltaHeading * 3.0, -0.3, 0.3); // Max 0.3 turn power



            /*
            forwardPower = deltaX * Math.cos(currentHeading) + deltaY * Math.sin(currentHeading);
            strafePower  = deltaX * Math.sin(currentHeading) + deltaY * Math.cos(currentHeading);

            // For this simple A to B movement, we aren't turning.
            turnPower = 0;

            // STEP 5: Normalize the powers
            maxPower = Math.max(1.0, Math.abs(forwardPower) + Math.abs(strafePower));
            forwardPower /= maxPower;
            strafePower /= maxPower;

            // STEP 6: Calculate the power for each of the four mecanum wheels.
            frontLeftPower  = (forwardPower + strafePower + turnPower) * moveSpeed;
            frontRightPower = (forwardPower - strafePower - turnPower) * moveSpeed;
            backLeftPower   = (forwardPower - strafePower + turnPower) * moveSpeed;
            backRightPower  = (forwardPower + strafePower - turnPower) * moveSpeed;

             */

            // STEP 7: Set the power on the motors.

            // Convert field-centric error to robot-centric movement commands
            // This is the key difference for mecanum/holonomic drive
            double angleRelativeToRobot = AngleUnit.RADIANS.normalize(absoluteAngleToTarget - currentHeading);
            double forward = Math.cos(angleRelativeToRobot) * drivePower;
            double strafe = Math.sin(angleRelativeToRobot) * drivePower;
            double turn = turnPower; // Use turn power to correct heading while moving

            // Apply mecanum wheel kinematics formula
            frontLeftPower = (forward + strafe + turn)*moveSpeed;
            frontRightPower = (forward - strafe - turn)*moveSpeed;
            backLeftPower = (forward - strafe + turn)*moveSpeed;
            backRightPower = (forward + strafe - turn)*moveSpeed;

            // Normalize wheel speeds to keep all powers within the [-1, 1] range
            maxPower = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            maxPower = Math.max(maxPower, Math.abs(backLeftPower));
            maxPower = Math.max(maxPower, Math.abs(backRightPower));

            if (maxPower > 1.0) {
                frontLeftPower /= maxPower;
                frontRightPower /= maxPower;
                backLeftPower /= maxPower;
                backRightPower /= maxPower;
            }

            // Set motor powers
            leftFrontDrive.setPower(frontLeftPower);
            rightFrontDrive.setPower(frontRightPower);
            leftBackDrive.setPower(backLeftPower);
            rightBackDrive.setPower(backRightPower);


            // Telemetry for debugging
            telemetry.addData("Current X", currentX);
            telemetry.addData("Current Y", currentY);
            telemetry.addData("Distance Left", distanceToTarget);
            telemetry.addData("Turn Error (Rad)", deltaHeading);
            telemetry.update();
            sleep(1000);

            // STEP 8: Provide telemetry for debugging.
            telemetry.addData("Target", String.format(Locale.US, "X: %.1f, Y: %.1f", target.getX(DistanceUnit.MM), target.getY(DistanceUnit.MM)));
            telemetry.addData("Current", String.format(Locale.US, "X: %.1f, Y: %.1f, H: %.1f", current.getX(DistanceUnit.MM), current.getY(DistanceUnit.MM), current.getHeading(AngleUnit.DEGREES)));
            telemetry.addData("Delta", String.format(Locale.US, "deltaX: %.1f, deltaY: %.1f", deltaX, deltaY));
            telemetry.addData("Powers of Motors", String.format(Locale.US, "FL: %.1f, FR: %.1f, BL: %.1f, BR: %.1f", frontLeftPower, frontRightPower, backLeftPower, backRightPower));
            telemetry.update();
            sleep(1000);

        }

        // Ensure motors are off if the opmode is stopped prematurely.
        stopRobot();
    }

    private boolean isAtTarget(double targetX, double targetY) {
        Pose2D currentPose = odo.getPosition();
        double dx = targetX - currentPose.getX(DistanceUnit.MM);
        double dy = targetY - currentPose.getY(DistanceUnit.MM);
        double distance = Math.sqrt(dx * dx + dy * dy);
        return distance < DISTANCE_TOLERANCE;
    }


    private void initialize(){
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
    }


    @Override
    public void runOpMode() {

        initialize();

        telemetry.addLine("Initialization Complete. Ready to start.");
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        resetRuntime();

        // --- AUTONOMOUS SEQUENCE ---
        // This sequence runs only ONCE after you press start.
        // --- AUTONOMOUS SEQUENCE ---
        // This sequence runs only ONCE after you press start.
        Pose2D target1 = null;

        if (opModeIsActive()) {

            // --- GO TO TARGET 1 ---
            telemetry.addLine("Driving to (X:0, Y:200)...");
            telemetry.update();

            // CORRECTED: Create a target pose to drive to X=0, Y=200, with a heading of 0 degrees.
            target1 = new Pose2D(DistanceUnit.MM,0, -200, AngleUnit.RADIANS,Math.toRadians(0));

            if (target1 != null) {
                // CORRECTED: Call the navigation method to drive there at 25% speed.
                telemetry.addLine("Navigate to (X:0, Y:200)...");
                telemetry.update();
                navigation(target1, 0.25);
            }else{
                telemetry.addLine("Target 1 is null.");
                telemetry.update();
                stopRobot();

            }

            telemetry.addLine("Arrived at Target 1.");
            telemetry.addData("Endpoints",target1);
            telemetry.update();
            sleep(1000); // Pause for 1 second

        }


        // The OpMode will automatically stop after the sequence above is complete.
    }

}
