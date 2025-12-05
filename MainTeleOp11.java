package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp(name = "MainTeleOp", group = "Competition")
public class MainTeleOp11 extends LinearOpMode {
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
    private Servo directionServo;

    private double launchPower;
    private double dirServoPos = 0.0; // Servo position starts at 0


    @Override
    public void runOpMode() {
        // --- INITIALIZATION PHASE ---
        // Map all hardware from the robot's configuration
        launchMotor = hardwareMap.get(DcMotor.class, "launchMotor");

        leftIntake = hardwareMap.get(DcMotor.class, "leftIntakeMotor");
        rightIntake = hardwareMap.get(DcMotor.class, "rightIntakeMotor");

        rightServo = hardwareMap.get(Servo.class, "rightServo");
        leftServo = hardwareMap.get(Servo.class, "leftServo");
        directionServo = hardwareMap.get(Servo.class, "tiltServo");


        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive"); //MOTORS - 0
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive"); // MOTORS - 1
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive"); // MOTORS - 2
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive"); // MOTORS - 3


        // --- SET MOTOR AND SERVO DIRECTIONS ---
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        launchMotor.setDirection(DcMotor.Direction.FORWARD);
        rightIntake.setDirection(DcMotor.Direction.REVERSE);
        leftIntake.setDirection(DcMotor.Direction.FORWARD);

        rightServo.setDirection(Servo.Direction.REVERSE);
        leftServo.setDirection(Servo.Direction.FORWARD);
        directionServo.setDirection(Servo.Direction.REVERSE);


        // --- SET MOTOR BEHAVIOR ---
        leftIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // --- SET MOTOR RUN MODES ---
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
            // --- Get Gamepad Input ---
            double turn = -gamepad2.right_stick_x;  // Turn
            double strafe = gamepad2.left_stick_x; // Left/Right
            double drive = -gamepad2.left_stick_y;  // Forward/Backward
            double servoPosition = gamepad2.left_trigger;
            boolean intakeOn = gamepad2.right_bumper;
            boolean reverseIntake = gamepad2.left_bumper;
            // The next line is no longer needed
            // double DirServoPos = gamepad2.right_trigger;

            // --- Gamepad 1 Launch Controls ---
            boolean triggerLaunch = gamepad1.right_trigger > 0.1; // Full power launch
            boolean reverseLaunch = gamepad1.left_trigger > 0.1;  // Reverse launch/intake


            // --- Launch Motor Control ---
            // 1. D-Pad fine-tuning for launch power (gamepad1)
            if (gamepad1.dpad_up) {
                launchPower += 0.01;
                sleep(50);
            } else if (gamepad1.dpad_down) {
                launchPower -= 0.01;
                sleep(50);
            }


            // 2. Preset Speeds (gamepad1)
            if (gamepad1.a) {
                launchPower = 0.42;
                dirServoPos=0.04;
            } else if (gamepad1.b) {
                launchPower = 0.0; // Off
                dirServoPos = 0.16;
            } else if (gamepad1.x) {
                launchPower = 0.72;
                dirServoPos = 0.22;
            } else if (gamepad1.y) {
                launchPower = 0.58;
                dirServoPos = 0.22;
            }
            // 3. Clamp the launchPower to be within a valid forward range [0, 1]
            launchPower = Math.max(0.0, Math.min(1.0, launchPower));
            if (gamepad1.dpad_right) {
                dirServoPos -= 0.02; // Increase position
                sleep(50); // Add a short delay to prevent rapid changes
            } else if (gamepad1.dpad_left) {
                dirServoPos += 0.02; // Decrease position
                sleep(50);
            }

            dirServoPos = Math.max(0.0, Math.min(1.0, dirServoPos));

            // 4. Determine the final power command with priorities
            double finalLaunchPower;
            if (triggerLaunch) {
                finalLaunchPower = 1.0; // HIGHEST PRIORITY: Full power launch
            } else if (reverseLaunch) {
                finalLaunchPower = -0.5; // SECOND PRIORITY: Reverse power
            } else {
                finalLaunchPower = launchPower; // DEFAULT: Use preset/tuned power
            }

            // --- Direction Servo Control (Gamepad 1 D-pad) ---
            if (gamepad1.dpad_right) {
                dirServoPos += 0.005; // Increase position
                sleep(50); // Add a short delay to prevent rapid changes
            } else if (gamepad1.dpad_left) {
                dirServoPos -= 0.005; // Decrease position
                sleep(50);
            }
            // Clamp the servo position to the valid range [0.0, 1.0]
            dirServoPos = Math.max(0.0, Math.min(1.0, dirServoPos));


            // --- Mecanum Drive Calculations ---
            double frontLeftPower = drive + strafe + turn;
            double frontRightPower = drive - strafe - turn;
            double backLeftPower = drive - strafe + turn;
            double backRightPower = drive + strafe - turn;


            // --- Set ALL Motor and Servo Powers ---
            // Drive motors
            leftFrontDrive.setPower(0.75 * frontLeftPower);
            rightFrontDrive.setPower(0.75 * frontRightPower);
            leftBackDrive.setPower(0.75 * backLeftPower);
            rightBackDrive.setPower(0.75 * backRightPower);

            // Set final launch motor power
            launchMotor.setPower(finalLaunchPower);

            // Intake Motor Logic
            if (reverseIntake){
                leftIntake.setPower(-0.25);
                rightIntake.setPower(-1);
            } else if (intakeOn) {
                leftIntake.setPower(0.25);
                rightIntake.setPower(1);
            } else {
                leftIntake.setPower(0);
                rightIntake.setPower(0);
            }


            // Control servos with the left trigger
            rightServo.setPosition(servoPosition);
            leftServo.setPosition(servoPosition);
            directionServo.setPosition(dirServoPos);



            // --- Telemetry ---
            telemetry.addData("Status", "Running");
            telemetry.addData("Launch Power Command", "%.2f", finalLaunchPower);
            telemetry.addData("Base Launch Power", "%.2f", launchPower);
            telemetry.addData("Intake On?", intakeOn);
            telemetry.addData("Reverse Intake?", reverseIntake);
            telemetry.addData("Trigger Launch?", triggerLaunch);
            telemetry.addData("Reverse Launch?", reverseLaunch);
            telemetry.addData("Servo Position", "%.2f", servoPosition);
            telemetry.addData("Direction Servo Position", "%.2f", dirServoPos);
            telemetry.addData("Drive", "%.2f, %.2f, %.2f", frontLeftPower, frontRightPower, backLeftPower, backRightPower);
            telemetry.update();
        }

        // Stop all motors on exit
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        launchMotor.setPower(0);
        leftIntake.setPower(0);
        rightIntake.setPower(0);
    }
}
