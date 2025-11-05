package org.firstinspires.ftc.teamcode;

// satvik and sai (ss squad) actually cooked and did something in this code.

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
// The @TeleOp name was "LaunchMotor", which is confusing. Renamed to "MainTeleOp" for clarity.
@TeleOp(name = "MainTeleOp", group = "Competition")
public class MainTeleOp extends LinearOpMode {

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
    private double launchPower = 0;
    private double finalLaunchPower;



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
        rightIntake.setDirection(DcMotor.Direction.FORWARD);
        leftIntake.setDirection(DcMotor.Direction.FORWARD);
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
        // --- TELEOP LOOP ---
        // --- TELEOP LOOP ---
        // --- TELEOP LOOP ---
        while (opModeIsActive()) {
            // --- Get Gamepad Input ---
            double drive = gamepad2.left_stick_y; // Inverted for standard FPS controls
            double strafe = -gamepad2.left_stick_x;
            double turn = gamepad2.right_stick_x;

            double servoPosition = gamepad2.left_trigger;
            boolean intakeOn = gamepad2.right_bumper;
            boolean reverseLaunch = gamepad2.a; // Button to reverse the launch motor

            // --- Launch Motor Control ---
            // This section determines the desired power for the launch motor.
            // We will set the actual motor power only ONCE at the end.

            // 1. D-Pad fine-tuning (Gamepad 1)
            // This adjusts the base 'launchPower' variable.
            if (gamepad1.dpad_up) {
                launchPower += 0.01; // Increase power
                sleep(50); // Small delay to make adjustments smoother
            } else if (gamepad1.dpad_down) {
                launchPower -= 0.01; // Decrease power
                sleep(50);
            }

            // 2. Preset Speeds (Gamepad 1)
            // These buttons set the 'launchPower' to a specific value.
            if (gamepad1.a) {
                launchPower = 0.40;
            } else if (gamepad1.b) {
                launchPower = 0.0; // Off
            } else if (gamepad1.x) {
                launchPower = 0.525;
            } else if (gamepad1.y) {
                launchPower = 0.60;
            }

            // 3. Clamp the base launchPower to be within a valid forward range [0, 1]
            if (launchPower > 1.0) {
                launchPower = 1.0;
            } else if (launchPower < 0.0) {
                launchPower = 0.0;
            }

            // 4. Check for variable trigger power (Gamepad 2)
            double triggerLaunchPower = gamepad2.right_trigger;

            // 5. Determine the final power command
            // Start with the base power set by d-pad/presets
            double finalLaunchPower = launchPower;
            // If the trigger is pressed, it overrides the base power
            if (triggerLaunchPower > 0.05) {
                finalLaunchPower = triggerLaunchPower;
            }

            // 6. HIGHEST PRIORITY: Check for reverse command
            // If 'a' on gamepad2 is pressed, override everything and run motor backwards.
            if (reverseLaunch) {
                finalLaunchPower = -0.5; // Set to a negative power for reverse
            }

            // --- Mecanum Drive Calculations ---
            double frontLeftPower = drive + strafe + turn;
            double frontRightPower = drive - strafe - turn;
            double backLeftPower = drive - strafe + turn;
            double backRightPower = drive + strafe - turn;

            // --- Set ALL Motor and Servo Powers ---
            // Drive motors at 50% speed
            leftFrontDrive.setPower(0.5 * frontLeftPower);
            rightFrontDrive.setPower(0.5 * frontRightPower);
            leftBackDrive.setPower(0.5 * backLeftPower);
            rightBackDrive.setPower(0.5 * backRightPower);

            // Set final launch motor power (this is the ONLY place we set it)
            launchMotor.setPower(finalLaunchPower);

            // Control servos with the left trigger
            rightServo.setPosition(servoPosition);
            leftServo.setPosition(servoPosition);

            // Control intake motors with the right bumper
            if (intakeOn) {
                leftIntake.setPower(0.5);
                rightIntake.setPower(1);
            } else {
                leftIntake.setPower(0);
                rightIntake.setPower(0);
            }

            // --- Telemetry ---
            telemetry.addData("Status", "Running");
            telemetry.addData("Final Launch Power", "%.2f", finalLaunchPower);
            telemetry.addData("Base (D-Pad/Preset) Power", "%.2f", launchPower);
            telemetry.addData("Intake On?", intakeOn);
            telemetry.addData("Reverse Active?", reverseLaunch);
            telemetry.update();
        }




        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
}


