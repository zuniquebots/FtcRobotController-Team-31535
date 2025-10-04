package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
// import com.qualcomm.robotcore.hardware.Servo; // Uncomment if you add servos back

@Autonomous(name = "Auto")
public class Auto extends LinearOpMode {

    // 1. Declare motor variables (do NOT initialize them to null)
    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;

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

        // --- 3. Set Motor Directions ---
        // This depends on your robot's build. You may need to reverse some of these.
        // A common mecanum setup is to reverse the motors on one side.
        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        // --- 4. Set Motor Run Modes and Behavior ---
        // Reset encoders to ensure they start at 0
        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Set motors to run based on power level (good for simple time-based driving)
        setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Set motors to brake when power is 0 to prevent coasting
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Signal that initialization is complete
        telemetry.addData("Status", "Initialized and ready to run");
        telemetry.update();

        // Wait for the driver to press the START button on the Driver Hub
        waitForStart();

        // --- 5. AUTONOMOUS SEQUENCE ---
        // The code below this line runs after START is pressed.
        if (opModeIsActive()) {
            // Example: Drive forward for 2 seconds
            driveForward(DRIVE_SPEED, 2000);

            // Example: Turn right for 1 second
            turnRight(TURN_SPEED, 1000);

            // Example: Strafe left for 1.5 seconds
            strafeLeft(DRIVE_SPEED, 1500);

            // Example: Drive backwards for 2 seconds
            driveForward(-DRIVE_SPEED, 2000); // Negative power to go backward
        }
    }

    /**
     * A helper method to set the run mode for all four drive motors at once.
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
     * @param power The power to set the motors (-1.0 to 1.0). Positive is forward.
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
     * @param power The power to set the motors (0 to 1.0).
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
     * @param power The power to set the motors (0 to 1.0).
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

    // You can add more methods here like strafeRight, turnLeft, etc.
}
