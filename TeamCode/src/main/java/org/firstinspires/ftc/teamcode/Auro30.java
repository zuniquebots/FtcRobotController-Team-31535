package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

// I've changed this to @Autonomous so you can run it as an auto program
@Autonomous(name = "Mecanum Odometry Navigation Demo", group = "Demo")
public class Auro30 extends LinearOpMode {

    // --- Robot constants (tune for your bot) ---
    static final double WHEEL_RADIUS = 0.048;     // meters (e.g., 48 mm wheels)
    static final double  TICKS_PER_REV = 537.7;        // goBilda 5202 encoders
    static final double GEAR_RATIO = 19.2;         // motor revs / wheel revs
    static final double L = 0.30;                 // Effective track width (Lx + Ly), meters. MUST BE TUNED.

    // --- Hardware ---
    DcMotor frontLeft, frontRight, backLeft, backRight;

    // --- Odometry state ---
    double x = 0.0;      // meters
    double y = 0.0;      // meters
    double theta = 0.0;  // radians

    // Previous encoder positions
    int prevFL = 0, prevFR = 0, prevBR = 0, prevBL = 0;

    @Override
    public void runOpMode() {
        // Init motors
        frontLeft  = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        frontRight = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        backLeft   = hardwareMap.get(DcMotor.class, "leftBackDrive");
        backRight  = hardwareMap.get(DcMotor.class, "rightBackDrive");

        // --- Set Motor Directions (CRUCIAL for Mecanum) ---
        // This is a common configuration, but you may need to adjust it for your robot.
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // --- Set Motor Behavior ---
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset encoders
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

        // --- AUTONOMOUS SEQUENCE ---
        if (opModeIsActive()) {
            // Create a background thread that will continuously update our odometry position.
            Thread odometryThread = new Thread(this::odometryLoop);
            odometryThread.start();

            // --- Your Autonomous Path Goes Here ---
            // Example: Drive to (X=0, Y=0.5 meters), then turn to 90 degrees, then drive to (X=0.5, Y=0.5)

            telemetry.addLine("Going to Corners");
            telemetry.update();
            driveToPosition(-0.5, 0, Math.toRadians(90), 0.3);


            // Stop the odometry thread when the autonomous path is finished.
            odometryThread.interrupt();
        }
    }

    /**
     * This method is designed to run in a background thread, continuously
     * updating the robot's position.t
     */
    private void odometryLoop() {
        while (opModeIsActive() && !Thread.currentThread().isInterrupted()) {
            updateOdometry();
            // We can add a small sleep here to not hog the CPU, but it's often not necessary.
        }
    }

    private void updateOdometry() {
        // Current encoder ticks
        int currFL = frontLeft.getCurrentPosition();
        int currFR = frontRight.getCurrentPosition();
        int currBR = backRight.getCurrentPosition();
        int currBL = backLeft.getCurrentPosition();

        // Δticks
        int dFL = currFL - prevFL;
        int dFR = currFR - prevFR;
        int dBR = currBR - prevBR;
        int dBL = currBL - prevBL;

        // Save for next loop
        prevFL = currFL;
        prevFR = currFR;
        prevBR = currBR;
        prevBL = currBL;

        // Convert ticks → distance (meters)
        // NOTE: The signs here might need to be inverted depending on how your encoders are wired.
        // If the robot moves forward and your Y value goes down, invert these.
        double distFL = ticksToMeters(dFL);
        double distFR = ticksToMeters(dFR);
        double distBR = ticksToMeters(dBR);
        double distBL = ticksToMeters(dBL);

        // Body-frame increments (robot's local movement)
        double dx_b = (distFL + distFR + distBR + distBL) / 4.0;
        double dy_b = (-distFL + distFR + distBR - distBL) / 4.0;
        double dTheta = (-distFL + distFR - distBR + distBL) / (4.0 * L);

        // Rotate into world frame (using midpoint heading for accuracy)
        double headingMid = theta + dTheta / 2.0;
        double dx_w = dx_b * Math.cos(headingMid) - dy_b * Math.sin(headingMid);
        double dy_w = dx_b * Math.sin(headingMid) + dy_b * Math.cos(headingMid);

        // Update global pose
        x += dx_w;
        y += dy_w;
        theta += dTheta;

        // Keep theta in [-pi, pi] for cleanliness
        theta = Math.atan2(Math.sin(theta), Math.cos(theta));
    }

    private double ticksToMeters(int dticks) {
        double wheelRevs = (dticks / TICKS_PER_REV) / GEAR_RATIO;
        return wheelRevs * 2.0 * Math.PI * WHEEL_RADIUS;
    }

    // =========================================================================================
    // --- NAVIGATION SYSTEM ---
    // =========================================================================================

    /**
     * The main navigation method. It drives the robot to a target position and orientation.
     * @param targetX The target X position in meters.
     * @param targetY The target Y position in meters.
     * @param targetTheta The target heading in radians.
     * @param maxPower The maximum power allowed for the motors (0.0 to 1.0).
     */
    private void driveToPosition(double targetX, double targetY, double targetTheta, double maxPower) {
        // --- PID Controller Constants (These need to be tuned!) ---
        final double Kp_pos = 2.0;  // Proportional gain for position. Determines how fast the robot tries to correct position errors.
        final double Kp_turn = 2.5; // Proportional gain for heading. Determines how fast the robot tries to correct heading errors.

        // --- Tolerances (How close is "close enough"?) ---
        final double posTolerance = 0.02;   // 2 cm
        final double thetaTolerance = Math.toRadians(2); // 2 degrees

        while (opModeIsActive()) {
            // --- Calculate Errors ---
            // Calculate the distance and angle errors in the world frame.
            double errorX = targetX - x;
            double errorY = targetY - y;
            double errorTheta = targetTheta - theta;

            // Normalize the heading error to be between -PI and PI
            errorTheta = Math.atan2(Math.sin(errorTheta), Math.cos(errorTheta));

            double distanceError = Math.hypot(errorX, errorY);

            // --- Check for Completion ---
            // If the robot is within the tolerance circles for both position and heading, we're done.
            if (distanceError < posTolerance && Math.abs(errorTheta) < thetaTolerance) {
                stopRobot();
                break; // Exit the loop
            }

            // --- Field-Centric to Robot-Centric Transformation ---
            // We need to translate the world-frame error (where we want to go on the field)
            // into robot-centric commands (how much to drive forward, strafe, and turn).
            // This is done by rotating the error vector by the negative of the robot's current heading.
            double robotX = errorX * Math.cos(-theta) - errorY * Math.sin(-theta);
            double robotY = errorX * Math.sin(-theta) + errorY * Math.cos(-theta);

            // --- Calculate Motor Powers using P-Controller ---
            double drivePower  = Range.clip(robotY * Kp_pos, -maxPower, maxPower);
            double strafePower = Range.clip(robotX * Kp_pos, -maxPower, maxPower);
            double turnPower   = Range.clip(-errorTheta * Kp_turn, -maxPower, maxPower); // Note the negative sign

            // --- Set Motor Powers ---
            setDrivePowers(drivePower, strafePower, turnPower);

            // --- Telemetry for Debugging ---
            telemetry.addData("Target", "X:%.2f, Y:%.2f, H:%.1f", targetX, targetY, Math.toDegrees(targetTheta));
            telemetry.addData("Current", "X:%.2f, Y:%.2f, H:%.1f", x, y, Math.toDegrees(theta));
            telemetry.addData("Error", "Dist:%.3f, Angle:%.1f", distanceError, Math.toDegrees(errorTheta));
            telemetry.addData("Power", "Drv:%.2f, Str:%.2f, Trn:%.2f", drivePower, strafePower, turnPower);
            telemetry.update();
        }
        stopRobot(); // Ensure robot is stopped after loop exits
    }

    /**
     * Helper method to set the power for all four mecanum drive motors.
     */
    private void setDrivePowers(double drive, double strafe, double turn) {
        double frontLeftPower  = drive + strafe + turn;
        double frontRightPower = drive - strafe - turn;
        double backLeftPower   = drive - strafe + turn;
        double backRightPower  = drive + strafe - turn;

        // Normalize motor powers if any of them exceed 1.0
        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower  /= max;
            frontRightPower /= max;
            backLeftPower   /= max;
            backRightPower  /= max;
        }

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }

    /**
     * Helper method to stop all drive motors.
     */
    private void stopRobot() {
        setDrivePowers(0, 0, 0);
    }
}
