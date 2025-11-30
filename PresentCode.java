package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp(name = "Present Code 1", group = "Competition")
public class PresentCode extends LinearOpMode {
    // --- Hardware Declarations ---
    private DcMotor launchMotor;
    private DcMotor leftIntake;
    private DcMotor rightIntake;
    private Servo rightServo;
    private Servo leftServo;
    // Drive motors are no longer declared
    private Servo directionServo;

    private double launchPower;
    private double dirServoPos = 0.0; // New variable for direction servo, starts at 0


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


        // --- Drive motor mapping removed ---


        // --- SET MOTOR AND SERVO DIRECTIONS ---
        // Drive motor direction settings removed

        launchMotor.setDirection(DcMotor.Direction.FORWARD);
        rightIntake.setDirection(DcMotor.Direction.FORWARD);
        leftIntake.setDirection(DcMotor.Direction.FORWARD);

        rightServo.setDirection(Servo.Direction.REVERSE);
        leftServo.setDirection(Servo.Direction.FORWARD);
        directionServo.setDirection(Servo.Direction.FORWARD);


        // --- SET MOTOR BEHAVIOR ---
        leftIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Drive motor ZeroPowerBehavior settings removed

        // --- SET MOTOR RUN MODES ---
        // Drive motor RunMode settings removed

        telemetry.addData("Status", "Initialized");
        telemetry.addData(">", "SS Squad cooked here.");
        telemetry.update();

        waitForStart();

        // --- TELEOP LOOP ---
        while (opModeIsActive()) {
            // --- Get Gamepad Input ---
            // Drive-related gamepad inputs are removed
            double servoPosition = gamepad2.left_trigger;
            boolean intakeOn = gamepad2.right_bumper;
            boolean reverseIntake = gamepad2.left_bumper;
            // The next line is no longer needed
            // double DirServoPos = gamepad2.right_trigger;

            // --- Gamepad 1 Launch Controls ---
            boolean triggerLaunch = gamepad1.right_trigger > 0.1; // Full power launch
            boolean reverseLaunch = gamepad1.left_trigger > 0.1;  // Reverse launch/intake



            // --- Launch Motor Control ---
            // 1. D-Pad fine-tuning (gamepad1)
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
            } else if (gamepad1.b) {
                launchPower = 0.0; // Off
            } else if (gamepad1.x) {
                launchPower = 0.55;
            } else if (gamepad1.y) {
                launchPower = 0.65;
            }

            // 3. Clamp the launchPower to be within a valid forward range [0, 1]
            launchPower = Math.max(0.0, Math.min(1.0, launchPower));


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


            // --- Mecanum Drive Calculations removed ---


            // --- Set ALL Motor and Servo Powers ---
            // Drive motor power settings removed

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


            // Control servos
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
            // Drive telemetry removed
            telemetry.update();
        }

        // Stop remaining motors on exit
        launchMotor.setPower(0);
        leftIntake.setPower(0);
        rightIntake.setPower(0);
    }
}
