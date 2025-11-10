package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

// Add the @Autonomous annotation to make this show up in the autonomous list
@Autonomous(name = "Field Centric Auto", group = "Linear OpMode")
public class FieldCentricMovementAuto extends LinearOpMode {
    // Hardware Declarations
    GoBildaPinpointDriver odo;
    private DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;

    // --- Constants for Navigation ---
    // How close the robot needs to be to the target to be considered "at the position"
    private static final double POSITION_TOLERANCE_MM = 20.0;
    // How close the robot's heading needs to be to the target heading to be "on heading"
    private static final double HEADING_TOLERANCE_DEGREES = 2.0;

    /**
     * Initializes all hardware, sets motor directions, and configures the odometry computer.
     */
    public void initialize() {
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "PinpointComputer");

        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");

        // Set motor directions (adjust these to match your robot's build)
        // This is a common configuration for mecanum drives.
        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set motor behavior when power is zero (BRAKE holds the position)
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Configure the Pinpoint Odometry computer
        odo.setOffsets(-84.0, -168.0, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        // Reset the position to (0,0) with a 0 heading and calibrate the IMU
        odo.resetPosAndIMU();

        telemetry.addLine("Initialization Complete. Ready to start.");
        telemetry.update();
    }

    /**
     * This is the main entry point for the OpMode.
     * It defines the sequence of autonomous actions.
     */
    @Override
    public void runOpMode() throws InterruptedException {
        // Step 1: Initialize all hardware
        initialize();

        waitForStart(); // Wait for the driver to press the START button

        // --- AUTONOMOUS PATH ---
        // This is where you define the robot's path by calling driveTo() for each target.
        if (opModeIsActive()) {

            // Example Path:
            // 1. Drive to X=500mm, Y=500mm, while facing 0 degrees
            telemetry.addLine("Driving to Target 1...");
            telemetry.update();
            driveTo(new Pose2D(DistanceUnit.MM,500, 500, AngleUnit.DEGREES,Math.toRadians(0)), 0.6); // 60% speed

            // 2. Turn on the spot to face 90 degrees (a left turn)
            telemetry.addLine("Turning to 90 degrees...");
            telemetry.update();
            driveTo(new Pose2D(DistanceUnit.MM,500, 500, AngleUnit.DEGREES,Math.toRadians(90)), 0.4); // 40% speed for turning

            // 3. Drive to the final target while holding the 90-degree heading
            telemetry.addLine("Driving to Final Target...");
            telemetry.update();
            driveTo(new Pose2D(DistanceUnit.MM,0, 1000,AngleUnit.DEGREES, Math.toRadians(90)), 0.6);

            // --- PATH END ---
            telemetry.addLine("Autonomous Path Finished!");
            telemetry.update();
            sleep(1000); // Pause for a second at the end
        }
    }

    /**
     * Drives the robot to a target Pose2D (X, Y, and Heading) using field-centric control.
     * This is a blocking method; it will not return until the robot reaches the target or the OpMode stops.
     *
     * @param targetPose The destination pose for the robot (X and Y in mm, Heading in radians).
     * @param moveSpeed  The maximum speed for the robot's movement (0.0 to 1.0).
     */
    public void driveTo(Pose2D targetPose, double moveSpeed) {
        // Proportional control constants. These may need tuning for your specific robot.
        final double P_DRIVE_GAIN = 0.005;  // For forward/backward movement
        final double P_STRAFE_GAIN = 0.007; // For sideways movement
        final double P_TURN_GAIN = 0.02;    // For turning movement

        while (opModeIsActive()) {
            // STEP 1: Get the robot's current position from the odometry computer
            odo.update();
            Pose2D currentPose = odo.getPosition();

            // STEP 2: Calculate the error between the target and the current position
            double errorX = targetPose.getX(DistanceUnit.MM) - currentPose.getX(DistanceUnit.MM);
            double errorY = targetPose.getY(DistanceUnit.MM) - currentPose.getY(DistanceUnit.MM);
            // Normalize the heading error to be between -PI and +PI for shortest turn
            double errorHeading = AngleUnit.normalizeRadians(targetPose.getHeading(AngleUnit.DEGREES) - currentPose.getHeading(AngleUnit.DEGREES));

            // STEP 3: Check if the robot has reached the target
            if (Math.hypot(errorX, errorY) < POSITION_TOLERANCE_MM &&
                    Math.abs(Math.toDegrees(errorHeading)) < HEADING_TOLERANCE_DEGREES) {
                stopMotors(); // Target reached, stop motors
                break;        // Exit the loop
            }

            // STEP 4: This is the core of Field-Centric control.
            // Rotate the field-centric error (errorX, errorY) into the robot's local coordinate frame.
            // This tells us how much to move "forward" and "sideways" from the robot's perspective.
            double robotAngleRad = currentPose.getHeading(AngleUnit.DEGREES);
            double forwardPower = (errorX * Math.cos(robotAngleRad) + errorY * Math.sin(robotAngleRad));
            double strafePower  = (-errorX * Math.sin(robotAngleRad) + errorY * Math.cos(robotAngleRad));

            // STEP 5: Calculate the final power for each component (drive, strafe, turn)
            double drive  = Range.clip(forwardPower * P_DRIVE_GAIN, -moveSpeed, moveSpeed);
            double strafe = Range.clip(strafePower * P_STRAFE_GAIN, -moveSpeed, moveSpeed);
            double turn   = Range.clip(-errorHeading * P_TURN_GAIN, -moveSpeed, moveSpeed); // Negative sign corrects the turn direction

            // STEP 6: Calculate the power for each mecanum wheel
            double frontLeftPower  = drive + strafe + turn;
            double frontRightPower = drive - strafe - turn;
            double backLeftPower   = drive - strafe + turn;
            double backRightPower  = drive + strafe + turn;

            // STEP 7: Normalize wheel powers to ensure they don't exceed 1.0
            double maxPower = Math.max(1.0, Math.abs(frontLeftPower));
            maxPower = Math.max(maxPower, Math.abs(frontRightPower));
            maxPower = Math.max(maxPower, Math.abs(backLeftPower));
            maxPower = Math.max(maxPower, Math.abs(backRightPower));

            // STEP 8: Set the power to the motors
            leftFrontDrive.setPower(frontLeftPower / maxPower);
            rightFrontDrive.setPower(frontRightPower / maxPower);
            leftBackDrive.setPower(backLeftPower / maxPower);
            rightBackDrive.setPower(backRightPower / maxPower);

            // Telemetry for Debugging
            telemetry.addData("Target", "X: %.1f, Y: %.1f, H: %.1f", targetPose.getX(DistanceUnit.MM), targetPose.getY(DistanceUnit.MM), Math.toDegrees(targetPose.getHeading(AngleUnit.DEGREES)));
            telemetry.addData("Current", "X: %.1f, Y: %.1f, H: %.1f", currentPose.getX(DistanceUnit.MM), currentPose.getY(DistanceUnit.MM), Math.toDegrees(currentPose.getHeading(AngleUnit.DEGREES)));
            telemetry.addData("Error", "X: %.1f, Y: %.1f, H: %.1f", errorX, errorY, Math.toDegrees(errorHeading));
            telemetry.update();
        }
        stopMotors(); // Ensure motors are off after the loop
    }

    /**
     * Helper method to stop all drive motors.
     */
    private void stopMotors() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
}
