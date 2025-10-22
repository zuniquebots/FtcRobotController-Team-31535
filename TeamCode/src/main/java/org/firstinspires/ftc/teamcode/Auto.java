package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
// import com.qualcomm.robotcore.hardware.Servo; // Uncomment if you add servos back

@Autonomous(name = "Auto")
public class Auto extends LinearOpMode {
    static final double WHEEL_RADIUS = 0.048;     // meters (e.g., 48 mm wheels)
    static final int TICKS_PER_REV = 1120;        // goBilda 5202 encoders
    static final double GEAR_RATIO = 1.0;         // motor revs / wheel revs
    static final double Lx = 0.15;                // meters, half length (front-back)
    static final double Ly = 0.150;
    static final double L = Lx + Ly;              // effective radius// meters, half width (side-to-side)

    double x = 0.0;      // meters
    double y = 0.0;      // meters
    double theta = 0.0;  // radians

    // Previous encoder positions
    int prevFL = 0, prevFR = 0, prevBR = 0, prevBL = 0;

    // 1. Declare motor variables (do NOT initialize them to null)
    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;
    private DcMotor launchMotor;
    private DcMotor leftIntakeMotor;
    private Servo rightServo;
    private Servo leftServo;

    // private DcMotor launchMotor; // Uncomment when you are ready to use them
    // private Servo rightServo;
    // private Servo leftServo;

    // Constants for robot dimensions and motor settings (optional but good practice)
    static final double COUNTS_PER_MOTOR_REV = 537.7; // Example: For a Gobilda 5203 series motor
    static final double DRIVE_GEAR_REDUCTION = 1.0;  // No external gearing
    static final double WHEEL_DIAMETER_INCHES = 3.78; // For a standard 96mm mecanum wheel
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.5;
    static final double TURN_SPEED = 0.4;


    @Override
    public void runOpMode() {
        // --- 2. INITIALIZATION PHASE ---
        // Map hardware from the robot's configuration file
        // IMPORTANT: Replace "left_front_drive", etc. with the exact names from your config
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");
        leftIntakeMotor = hardwareMap.get(DcMotor.class, "leftIntake");


        // --- 3. Set Motor Directions ---
        // This depends on your robot's build. You may need to reverse some of these.
        // A common mecanum setup is to reverse the motors on one side.
        leftFrontDrive.setDirection(REVERSE);
        leftBackDrive.setDirection(REVERSE);
        rightFrontDrive.setDirection(FORWARD);
        rightBackDrive.setDirection(FORWARD);
        leftIntakeMotor.setDirection(FORWARD);

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftIntakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Signal that initialization is complete
        telemetry.addData("Status", "Initialized and ready to run");
        telemetry.update();

        // Wait for the driver to press the START button on the Driver Hub
        waitForStart();

        // --- 5. AUTONOMOUS SEQUENCE ---
        // The code below this line runs after START is pressed.
        while (opModeIsActive()) {
            updateOdometry();
            telemetry.addData("x (m)", x);
            telemetry.addData("y (m)", y);
            telemetry.addData("theta (deg)", Math.toDegrees(theta));
            telemetry.update();
        }
    }

    /**
     * A helper method to set the run mode for all four drive motors at once.
     *
     * @param mode The DcMotor.RunMode to set.
     */
    private void setMotorMode(DcMotor.RunMode mode) {
        leftFrontDrive.setMode(mode);
        rightFrontDrive.setMode(mode);
        leftBackDrive.setMode(mode);
        rightBackDrive.setMode(mode);
    }

    /**
     * A helper method to set the zero power behavior for all drive motors.
     *
     * @param behavior The DcMotor.ZeroPowerBehavior to set.
     */
    private void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        leftFrontDrive.setZeroPowerBehavior(behavior);
        rightFrontDrive.setZeroPowerBehavior(behavior);
        leftBackDrive.setZeroPowerBehavior(behavior);
        rightBackDrive.setZeroPowerBehavior(behavior);
    }

    /**
     * Stops all drive motors.
     */
    private void stopDriving() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    /**
     * Drives the robot forward or backward for a specified time and power.
     *
     * @param power  The power to set the motors (-1.0 to 1.0). Positive is forward.
     * @param timeMs The time in milliseconds to drive.
     */
    private void driveForward(double power, long timeMs) {
        if (!opModeIsActive()) return; // Safety check

        leftFrontDrive.setPower(power);
        rightFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightBackDrive.setPower(power);

        sleep(timeMs);

        stopDriving();
    }

    /**
     * Turns the robot right (clockwise) for a specified time and power.
     *
     * @param power  The power to set the motors (0 to 1.0).
     * @param timeMs The time in milliseconds to turn.
     */
    private void turnRight(double power, long timeMs) {
        if (!opModeIsActive()) return;

        // To turn right, left wheels go forward, right wheels go backward
        leftFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightFrontDrive.setPower(-power);
        rightBackDrive.setPower(-power);

        sleep(timeMs);

        stopDriving();
    }

    /**
     * Strafes the robot left for a specified time and power.
     *
     * @param power  The power to set the motors (0 to 1.0).
     * @param timeMs The time in milliseconds to strafe.
     */
    private void strafeLeft(double power, long timeMs) {
        if (!opModeIsActive()) return;

        // To strafe left: FL backward, FR forward, BL forward, BR backward
        leftFrontDrive.setPower(-power);
        rightFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightBackDrive.setPower(-power);

        sleep(timeMs);

        stopDriving();
    }

    private void updateOdometry() {
        // Current encoder ticks
        int currFL = leftFrontDrive.getCurrentPosition();
        int currFR = rightFrontDrive.getCurrentPosition();
        int currBR = rightBackDrive.getCurrentPosition();
        int currBL = leftBackDrive.getCurrentPosition();

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

    private void launchMotor(long timeMS) {
        launchMotor = hardwareMap.get(DcMotor.class, "launchMotor");

        launchMotor.setDirection(FORWARD);

        launchMotor.setPower(0.42);
        sleep(timeMS);
        launchMotor.setPower(0);
    }

    private void launchServo() {
        leftServo = hardwareMap.get(Servo.class, "leftServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");


        leftServo.setDirection(Servo.Direction.FORWARD);
        rightServo.setDirection(Servo.Direction.REVERSE);

        leftServo.setPosition(1);
        rightServo.setPosition(1);
        sleep(3000);
        leftServo.setPosition(0);
        rightServo.setPosition(0);

    }

    private void launchIntake(long timeMS) {
        leftIntakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        launchMotor.setDirection(FORWARD);

        launchMotor.setPower(0.42);
        sleep(timeMS);
        launchMotor.setPower(0);
    }

    private void odometryLoop() {
        while (opModeIsActive() && !Thread.currentThread().isInterrupted()) {
            updateOdometry();
        }


        // You can add more methods here like strafeRight, turnLeft, etc.
    }

    private void Navigation(double targetX, double targetY, double targetTheta) {
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
                stopDriving();
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


            // --- Telemetry for Debugging ---
            telemetry.addData("Target", "X:%.2f, Y:%.2f, H:%.1f", targetX, targetY, Math.toDegrees(targetTheta));
            telemetry.addData("Current", "X:%.2f, Y:%.2f, H:%.1f", x, y, Math.toDegrees(theta));
            telemetry.addData("Error", "X:%.2f, Y:%.2f, H:%.1f", distanceError, Math.toDegrees(errorTheta));
            telemetry.addData("Power", "Drv:%.2f, Str:%.2f, Trn:%.2f", DRIVE_SPEED, DRIVE_SPEED, TURN_SPEED);
            telemetry.update();
        }
    }

    private void setDrivePowers(double drive, double strafe, double turn) {
        double frontLeftPower = drive + strafe + turn;
        double frontRightPower = drive - strafe - turn;
        double backLeftPower = drive - strafe + turn;
        double backRightPower = drive + strafe - turn;

        // Normalize motor powers if any of them exceed 1.0
        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        leftFrontDrive.setPower(frontLeftPower);
        rightFrontDrive.setPower(frontRightPower);
        leftBackDrive.setPower(backLeftPower);
        rightBackDrive.setPower(backRightPower);
    }
}

