package org.firstinspires.ftc.teamcode;

// satvik and sai (ss squad) actually cooked and did something in this code.

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

// The @TeleOp name was "LaunchMotor", which is confusing. Renamed to "MainTeleOp" for clarity.
@TeleOp(name = "MainTeleOp", group = "Competition")
public class MainTeleOp extends LinearOpMode {

    // --- Constants for Odometry (if you decide to use it) ---
    static final double WHEEL_RADIUS = 0.048;     // meters (e.g., 48 mm wheels)
    static final int TICKS_PER_REV = 1120;        // goBilda 5202 encoders
    static final double GEAR_RATIO = 1.0;         // motor revs / wheel revs
    static final double L = 0.30;                 // Effective track width (Lx + Ly), in meters. MUST BE TUNED.

    // --- Odometry state variables ---
    double x_pos = 0.00;      // meters
    double y_pos = 0.0;      // meters
    double theta_pos = 0.0;  // radians

    // Previous encoder positions
    int prevFL = 0, prevFR = 0, prevBR = 0, prevBL = 0;

    // --- Hardware Declarations ---
    private DcMotor launchMotor;
    private DcMotor leftIntake;
    private DcMotor rightIntake;
    private Servo rightServo;
    private Servo leftServo;
    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;

    @Override
    public void runOpMode() {
        // --- INITIALIZATION PHASE ---
        // Map all hardware from the robot's configuration
        launchMotor = hardwareMap.get(DcMotor.class, "launchMotor");
        leftIntake = hardwareMap.get(DcMotor.class, "leftIntakeMotor");
        rightIntake = hardwareMap.get(DcMotor.class, "rightIntakeMotor");
        rightServo = hardwareMap.get(Servo.class, "rightServo");
        leftServo = hardwareMap.get(Servo.class, "leftServo");
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");
        // The second mapping of launchMotor was removed.

        // --- SET MOTOR AND SERVO DIRECTIONS ---
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        launchMotor.setDirection(DcMotor.Direction.FORWARD);
        rightIntake.setDirection(DcMotor.Direction.REVERSE);
        leftIntake.setDirection(DcMotor.Direction.REVERSE);
        rightServo.setDirection(Servo.Direction.REVERSE);
        leftServo.setDirection(Servo.Direction.FORWARD);

        // --- SET MOTOR BEHAVIOR ---
        // Set zero power behavior to BRAKE for more immediate stops
        leftIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Reset encoders and set run mode for odometry
        //sai and satvik coded all of this bro trust
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.addData(">", "SS Squad cooked here.");
        telemetry.update();

        waitForStart();

        // --- TELEOP LOOP ---
        while (opModeIsActive()) {
            // --- Update Odometry ---
            // This continuously tracks the robot's position in the background
            updateOdometry();

            // --- Get Gamepad Input ---
            double drive = gamepad2.left_stick_y; // Inverted for standard FPS controls
            double strafe = -gamepad2.left_stick_x;
            double turn = gamepad2.right_stick_x;

            double launch = gamepad2.right_trigger;
            double servoPosition = gamepad2.left_trigger;
            boolean isIntakeRunning = gamepad2.right_bumper;

            // --- Mecanum Drive Calculations ---
            double frontLeftPower = drive + strafe + turn;
            double frontRightPower = drive - strafe - turn;
            double backLeftPower = drive - strafe + turn;
            double backRightPower = drive + strafe - turn;

            // --- Set Motor and Servo Powers ---
            launchMotor.setPower(-0.43*launch);
            leftFrontDrive.setPower(0.5 * frontLeftPower);
            rightFrontDrive.setPower(0.5 * frontRightPower);
            leftBackDrive.setPower(0.5 * backLeftPower);
            rightBackDrive.setPower(0.5 * backRightPower);

            // Control servos with the left trigger
            rightServo.setPosition(servoPosition);
            leftServo.setPosition(servoPosition);

            // Control intake motors with an if-else statement
            if (isIntakeRunning) {
                leftIntake.setPower(-0.5);
                rightIntake.setPower(-1.0);
            } else {
                leftIntake.setPower(0.0);
                rightIntake.setPower(0.0);
            }

            // --- Telemetry ---
            telemetry.addData("Status", "Running");
            telemetry.addData("Front Left", leftFrontDrive.getPower());
            telemetry.addData("Back Left", leftBackDrive.getPower());
            telemetry.addData("Front Right", rightFrontDrive.getPower());
            telemetry.addData("Back Right", rightBackDrive.getPower());
            telemetry.addData("Launch Motor", launchMotor.getPower());
            telemetry.addData("Odometry X:", x_pos);
            telemetry.addData("Odometry Y:", y_pos);
            telemetry.addData("Odometry Theta:", Math.toDegrees(theta_pos));
            telemetry.addData("Intake Running?", isIntakeRunning);
            telemetry.update();
        }

        // --- POST-LOOP CLEANUP ---
        // Ensure all motors are stopped when the OpMode ends.
        launchMotor.setPower(0);
        leftIntake.setPower(0);
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    } // End of runOpMode()

    // =========================================================================================
    // HELPER METHODS - MUST BE OUTSIDE OF runOpMode()
    // =========================================================================================

    /**
     * Updates the robot's position (x, y, theta) based on encoder changes.
     * This method uses the mecanum drive kinematics for odometry.
     */
    private void updateOdometry() {
        // Current encoder ticks
        int currFL = leftFrontDrive.getCurrentPosition();
        int currFR = rightFrontDrive.getCurrentPosition();
        int currBR = rightBackDrive.getCurrentPosition();
        int currBL = leftBackDrive.getCurrentPosition();

        // Δticks since last update
        int dFL = currFL - prevFL;
        int dFR = currFR - prevFR;
        int dBR = currBR - prevBR;
        int dBL = currBL - prevBL;

        // Save current ticks for the next loop
        prevFL = currFL;
        prevFR = currFR;
        prevBR = currBR;
        prevBL = currBL;

        // Convert ticks to distance in meters
        double distFL = ticksToMeters(dFL);
        double distFR = ticksToMeters(dFR);
        double distBR = ticksToMeters(dBR);
        double distBL = ticksToMeters(dBL);

        // Calculate body-frame increments (robot's local movement)
        // Note: The signs depend on motor directions and kinematics.
        // This is a standard formulation.
        double dx_robot = (distFL + distFR + distBR + distBL) / 4.0;
        double dy_robot = (-distFL + distFR - distBR + distBL) / 4.0; // This seems incorrect for strafing
        double dTheta = (-distFL + distFR - distBR + distBL) / (4.0 * L);

        // A more standard strafe calculation would be:
        // double dy_robot = (-distFL + distFR + distBR - distBL) / 4.0;

        // Rotate movement into the world frame (global coordinates)
        double headingMid = theta_pos + dTheta / 2.0;
        double dx_world = dx_robot * Math.cos(headingMid) - dy_robot * Math.sin(headingMid);
        double dy_world = dx_robot * Math.sin(headingMid) + dy_robot * Math.cos(headingMid);

        // Update global pose
        x_pos += dx_world;
        y_pos += dy_world;
        theta_pos += dTheta;

        // Keep theta within [-pi, pi] for consistency
        theta_pos = Math.atan2(Math.sin(theta_pos), Math.cos(theta_pos));
    }

    /**
     * Converts encoder ticks to meters.
     */
    private double ticksToMeters(int dticks) {
        double wheelRevs = (dticks / (double) TICKS_PER_REV) / GEAR_RATIO;
        return wheelRevs * 2.0 * Math.PI * WHEEL_RADIUS;
    }
}
