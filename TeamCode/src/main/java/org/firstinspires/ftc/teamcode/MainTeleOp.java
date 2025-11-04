package org.firstinspires.ftc.teamcode;

// satvik and sai (ss squad) actually cooked and did something in this code.

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

// The @TeleOp name was "LaunchMotor", which is confusing. Renamed to "MainTeleOp" for clarity.
@TeleOp(name = "MainTeleOp", group = "Competition")
public class MainTeleOp extends LinearOpMode {


    // --- Odometry state variables ---



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
        while (opModeIsActive()) {
            // --- Get Gamepad Input ---
            double drive = gamepad2.left_stick_y; // Inverted for standard FPS controls
            double strafe = -gamepad2.left_stick_x;
            double turn = gamepad2.right_stick_x;

            double servoPosition = gamepad2.left_trigger;
            boolean intakeOn = gamepad2.right_bumper; // Renamed for clarity
            boolean reverseLaunch = gamepad2.a; // Renamed for clarity


            // --- Launch Motor Control ---
            // The right trigger on gamepad2 can be used for variable speed control.
            double triggerLaunchPower = gamepad2.right_trigger;

            // Use d-pad on gamepad1 to fine-tune the launchPower variable.
            // This allows setting a "base" power that the trigger can override.
            if (gamepad1.dpad_up) {
                launchPower -= 0.02; // Increase power
                sleep(50); // Add a small delay to prevent it from flying up too fast
            } else if (gamepad1.dpad_down) {
                launchPower += 0.02; // Decrease power
                sleep(50); // Add a small delay
            }
            if (gamepad1.a) {
                finalLaunchPower = -0.4; // Decrease power
            }
            if (gamepad1.b)
                finalLaunchPower = 0.0;
            if (gamepad1.x) {
                finalLaunchPower = -0.525;
            if (gamepad1.y){
                finalLaunchPower = -0.6;
            }
            if (reverseLaunch) {
                launchMotor.setPower(-0.5);
            } else {
                continue;
            }
        }

            // --- Clamp the launchPower to be between 0 and 1 ---
            if (launchPower > 1.0) {
                launchPower = 1.0;
            } else if (launchPower < 0.0) {
                launchPower = 0.0;
            }

            // Determine the final power. If the trigger is pressed, it overrides the d-pad setting.
            // Otherwise, it uses the d-pad setting.
            double finalLaunchPower = (triggerLaunchPower > 0.05) ? triggerLaunchPower : launchPower;


            // --- Mecanum Drive Calculations ---
            double frontLeftPower = drive + strafe + turn;
            double frontRightPower = drive - strafe - turn;
            double backLeftPower = drive - strafe + turn;
            double backRightPower = drive + strafe - turn;

            // --- Set Motor and Servo Powers ---
            // Drive motors at 50% speed
            leftFrontDrive.setPower(0.5 * frontLeftPower);
            rightFrontDrive.setPower(0.5 * frontRightPower);
            leftBackDrive.setPower(0.5 * backLeftPower);
            rightBackDrive.setPower(0.5 * backRightPower);

            // Set final launch motor power
            launchMotor.setPower(finalLaunchPower);

            // Control servos with the left trigger
            rightServo.setPosition(servoPosition);
            leftServo.setPosition(servoPosition);

            // Control intake motors with the right bumper
            if (intakeOn) {
                leftIntake.setPower(1);
                rightIntake.setPower(0.5);
            } else {
                leftIntake.setPower(0);
                rightIntake.setPower(0);
            }

            // --- Telemetry ---
            telemetry.addData("Status", "Running");
            telemetry.addData("Drive Speed", "50%");
            telemetry.addData("D-Pad Launch Power", "%.2f", launchPower);
            telemetry.addData("Trigger Launch Power", "%.2f", triggerLaunchPower);
            telemetry.addData("Final Launch Power", "%.2f", finalLaunchPower);
            telemetry.addData("Intake On?", intakeOn);
            telemetry.update();
        }



        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
}


