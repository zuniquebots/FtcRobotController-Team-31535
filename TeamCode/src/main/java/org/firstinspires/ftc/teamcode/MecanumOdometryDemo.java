package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

public class MecanumOdometryDemo extends LinearOpMode {

    // --- Robot constants (tune for your bot) ---
    static final double WHEEL_RADIUS = 0.048;     // meters (e.g., 48 mm wheels)
    static final int TICKS_PER_REV = 1120;        // goBilda 5202 encoders
    static final double GEAR_RATIO = 1.0;         // motor revs / wheel revs
    static final double Lx = 0.15;                // meters, half length (front-back)
    static final double Ly = 0.15;                // meters, half width (side-to-side)
    static final double L = Lx + Ly;              // effective radius

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
        frontLeft = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        frontRight = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        backLeft = hardwareMap.get(DcMotor.class, "leftBackDrive");
        backRight = hardwareMap.get(DcMotor.class, "rightBackDrive");

        // Reset encoders
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            updateOdometry();

            telemetry.addData("x (m)", x);
            telemetry.addData("y (m)", y);
            telemetry.addData("theta (deg)", Math.toDegrees(theta));
            telemetry.update();
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
        double distFL = ticksToMeters(dFL);
        double distFR = ticksToMeters(dFR);
        double distBR = ticksToMeters(dBR);
        double distBL = ticksToMeters(dBL);

        // Body-frame increments
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
        double wheelRevs = (dticks / (double) TICKS_PER_REV) / GEAR_RATIO;
        return wheelRevs * 2.0 * Math.PI * WHEEL_RADIUS;
    }

    private void navigation(double targetX, double targetY, double targetTheta) {
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
                //stopRobot(); our own version of stopRobot
                telemetry.addData("Status", "Reached target");
                telemetry.update();
                break; // Exit the loop
            }

            // --- Field-Centric to Robot-Centric Transformation ---
            // We need to translate the world-frame error (where we want to go on the field)
            // into robot-centric commands (how much to drive forward, strafe, and turn).
            // This is done by rotating the error vector by the negative of the robot's current heading.
            double robotX = errorX * Math.cos(-theta) - errorY * Math.sin(-theta);
            double robotY = errorX * Math.sin(-theta) + errorY * Math.cos(-theta);
            int var=0;


            // --- Telemetry for Debugging ---
            telemetry.addData("Target", "X:%.2f, Y:%.2f, H:%.1f", targetX, targetY, Math.toDegrees(targetTheta));
            telemetry.addData("Current", "X:%.2f, Y:%.2f, H:%.1f", x, y, Math.toDegrees(theta));
            telemetry.addData("Error", "X:%.2f, Y:%.2f, H:%.1f", distanceError, Math.toDegrees(errorTheta));
            telemetry.addData("Power", "Drv:%.2f, Str:%.2f, Trn:%.2f",var);
            telemetry.update();
        }
        //stopRobot(); our own version of stopRobot


        }
    }
