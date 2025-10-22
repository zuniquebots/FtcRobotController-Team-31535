package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name = "Odometry Navigation Base", group = "Demos")
public class OdometryDemos extends LinearOpMode {

    // --- ROBOT CONSTANTS (Tune these for your specific robot) ---
    static final double TICKS_PER_REV = 1120;
    static final double WHEEL_DIAMETER_METERS = 0.096;
    static final double GEAR_RATIO = 1.0;
    static final double TICKS_PER_METER = (TICKS_PER_REV * GEAR_RATIO) / (WHEEL_DIAMETER_METERS * Math.PI);

    // This is the effective distance between your left and right odometry wheels.
    // It's crucial for accurate turning. You MUST tune this value.
    static final double TRACK_WIDTH_METERS = 0.3;

    // --- HARDWARE DECLARATIONS ---
    DcMotor frontLeft, frontRight, backLeft, backRight;

    // --- ODOMETRY STATE (Global Position) ---
    double robotX = 0.0;      // meters
    double robotY = 0.0;      // meters
    double robotTheta = 0.0;  // radians

    // Previous encoder values for calculating deltas
    int prevFL = 0, prevFR = 0, prevBL = 0, prevBR = 0;

    @Override
    public void runOpMode() {
        // --- INITIALIZATION ---
        frontLeft  = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        frontRight = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        backLeft   = hardwareMap.get(DcMotor.class, "leftBackDrive");
        backRight  = hardwareMap.get(DcMotor.class, "rightBackDrive");

        // Set motor directions (CRUCIAL! You must test and verify this for your robot)
        // A common configuration is to reverse one side.
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set motors to brake when power is 0 for more precise stops
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset encoders and set to run without them for power-based control
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Initialization Complete. Ready to start.");
        telemetry.update();

        waitForStart();

        // --- AUTONOMOUS EXECUTION ---
        if (opModeIsActive()) {
            // Start a background thread to continuously update the robot's position
            Thread odometryThread = new Thread(this::odometryLoop);
            odometryThread.start();

            // --- DEFINE YOUR AUTONOMOUS PATH HERE ---
            // Example: Drive forward 0.5 meters, turn 90 degrees, then strafe 0.5 meters right.

            telemetry.addLine("Driving to (0, 0.5)");
            telemetry.update();
            driveToPosition(0, 0.5, 0, 0.5);

            telemetry.addLine("Turning to 90 degrees");
            telemetry.update();
            driveToPosition(0, 0.5, Math.toRadians(90), 0.4);

            telemetry.addLine("Driving to (-0.5, 0.5)");
            telemetry.update();
            driveToPosition(-0.5, 0.5, Math.toRadians(90), 0.5);

            // Stop the odometry thread at the end of the autonomous period
            odometryThread.interrupt();
        }
    }

    /**
     * This method runs in a background thread to update the robot's position
     * as fast as possible.
     */
    private void odometryLoop() {
        while (opModeIsActive() && !Thread.currentThread().isInterrupted()) {
            updateOdometry();
        }
    }

    /**
     * This is the core of the odometry system. It reads encoder values and
     * updates the robot's global (x, y, theta) position.
     */
    public void updateOdometry() {
        // Read current encoder positions
        int currFL = frontLeft.getCurrentPosition();
        int currFR = frontRight.getCurrentPosition();
        int currBL = backLeft.getCurrentPosition();
        int currBR = backRight.getCurrentPosition();

        // Calculate the change in ticks since the last update
        // IMPORTANT: If your encoders count backwards, you may need to flip the sign here (e.g., -dFR)
        double deltaTicksFL = currFL - prevFL;
        double deltaTicksFR = currFR - prevFR;
        double deltaTicksBL = currBL - prevBL;
        double deltaTicksBR = currBR - prevBR;

        // Calculate the change in robot heading (orientation)
        double dtheta = ((deltaTicksFR - deltaTicksFL) / TICKS_PER_METER) / TRACK_WIDTH_METERS;

        // Calculate the change in the robot's forward and strafe position (robot-centric)
        double dx_robot = ((deltaTicksFL + deltaTicksFR) / 2.0) / TICKS_PER_METER;
        double dy_robot = ((deltaTicksBL + deltaTicksBR) / 2.0) / TICKS_PER_METER; // Assuming strafe pods

        // Rotate the robot-centric changes into the global coordinate frame
        double headingMid = robotTheta + dtheta / 2.0;
        double dx_world = dx_robot * Math.cos(headingMid) - dy_robot * Math.sin(headingMid);
        double dy_world = dx_robot * Math.sin(headingMid) + dy_robot * Math.cos(headingMid);

        // Update the global position
        robotX += dx_world;
        robotY += dy_world;
        robotTheta += dtheta;

        // Save the current encoder values for the next loop
        prevFL = currFL;
        prevFR = currFR;
        prevBL = currBL;
        prevBR = currBR;
    }

    /**
     * Drives the robot to a target coordinate on the field.
     * @param targetX Target X position (meters)
     * @param targetY Target Y position (meters)
     * @param targetTheta Target heading (radians)
     * @param maxPower Maximum motor power (0.0 to 1.0)
     */
    public void driveToPosition(double targetX, double targetY, double targetTheta, double maxPower) {
        // --- P-Controller Constants (Tune these) ---
        final double Kp_pos = 1.5;  // Proportional gain for position
        final double Kp_turn = 2.0; // Proportional gain for turning

        // --- Tolerances (How close is "close enough"?) ---
        final double posTolerance = 0.02;   // 2 cm
        final double thetaTolerance = Math.toRadians(2); // 2 degrees

        while (opModeIsActive()) {
            // Calculate errors in the global frame
            double errorX = targetX - robotX;
            double errorY = targetY - robotY;
            double errorTheta = targetTheta - robotTheta;
            errorTheta = Math.atan2(Math.sin(errorTheta), Math.cos(errorTheta)); // Normalize angle

            // Check if the robot has reached the target
            if (Math.hypot(errorX, errorY) < posTolerance && Math.abs(errorTheta) < thetaTolerance) {
                stopRobot();
                break;
            }

            // Rotate the global error into the robot's local frame
            double robotCentricX = errorX * Math.cos(-robotTheta) - errorY * Math.sin(-robotTheta);
            double robotCentricY = errorX * Math.sin(-robotTheta) + errorY * Math.cos(-robotTheta);

            // Calculate motor powers using a simple P-controller
            double drivePower  = Range.clip(robotCentricY * Kp_pos, -maxPower, maxPower);
            double strafePower = Range.clip(robotCentricX * Kp_pos, -maxPower, maxPower);
            double turnPower   = Range.clip(-errorTheta * Kp_turn, -maxPower, maxPower);

            setDrivePowers(drivePower, strafePower, turnPower);

            // Telemetry for debugging
            telemetry.addData("Target", "X:%.2f, Y:%.2f, H:%.1f", targetX, targetY, Math.toDegrees(targetTheta));
            telemetry.addData("Current", "X:%.2f, Y:%.2f, H:%.1f", robotX, robotY, Math.toDegrees(robotTheta));
            telemetry.update();
        }
        stopRobot();
    }

    /** Helper method to set powers for a mecanum drive. */
    public void setDrivePowers(double drive, double strafe, double turn) {
        double FL = drive + strafe + turn;
        double FR = drive - strafe - turn;
        double BL = drive - strafe + turn;
        double BR = drive + strafe - turn;

        // Normalize powers if any exceed 1.0
        double max = Math.max(1.0, Math.abs(FL));
        max = Math.max(max, Math.abs(FR));
        max = Math.max(max, Math.abs(BL));
        max = Math.max(max, Math.abs(BR));

        frontLeft.setPower(FL / max);
        frontRight.setPower(FR / max);
        backLeft.setPower(BL / max);
        backRight.setPower(BR / max);
    }

    /** Helper method to stop all drive motors. */
    public void stopRobot() {
        setDrivePowers(0, 0, 0);
    }


    public void calculateOdometry(double targetX, double targetY) {
            final double Kp = 1.2;          // Proportional gain
            final double maxPower = 0.5;    // Limit motor power
            final double tolerance = 0.02;  // Stop if within 2cm

            while (opModeIsActive()) {
                // Calculate the error (distance to target)
                double deltaX = targetX - robotX;
                double deltaY = targetY - robotY;

                double distance = Math.hypot(deltaX, deltaY);
                if (distance < tolerance) {
                    break;
                }

                // Convert global direction into robot-relative direction
                double angleToTarget = Math.atan2(deltaY, deltaX);
                double robotRelativeAngle = angleToTarget - robotTheta;

                double drivePower = Math.cos(robotRelativeAngle) * Kp;
                double strafePower = Math.sin(robotRelativeAngle) * Kp;

                // Clip power
                drivePower = Range.clip(drivePower, -maxPower, maxPower);
                strafePower = Range.clip(strafePower, -maxPower, maxPower);

                // Apply powers (no turning in this version)
                setDrivePowers(drivePower, strafePower, 0);

                // Optional: Telemetry
                telemetry.addData("GoTo", "X: %.2f, Y: %.2f", targetX, targetY);
                telemetry.addData("Pos", "X: %.2f, Y: %.2f", robotX, robotY);
                telemetry.update();



            }

            stopRobot(); // Kill power at end
        }

    }
