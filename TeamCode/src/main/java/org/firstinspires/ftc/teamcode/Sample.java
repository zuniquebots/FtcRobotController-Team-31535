package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "klip Motor", group = "Examples")
public class Sample extends LinearOpMode {
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftFrontDrive = null;
    private DcMotor launchMotor = null;
    private Servo rightServo = null;
    private Servo leftServo = null;

    @Override
    public void runOpMode() {
        // Initialize hardware
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class,"leftBackDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class,"rightBackDrive");
        launchMotor = hardwareMap.get(DcMotor.class, "launchMotor");
        rightServo = hardwareMap.get(Servo.class, "rightServo");
        leftServo = hardwareMap.get(Servo.class, "leftServo");

        // Set motor directions (adjust as needed)
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        launchMotor.setDirection(DcMotor.Direction.FORWARD);
        rightServo.setDirection(Servo.Direction.FORWARD);
        leftServo.setDirection(Servo.Direction.REVERSE);


        // Send telemetry data to Driver Station
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Get gamepad input
            double drive = gamepad2.right_stick_y;
            double turn = gamepad2.right_stick_x;
            double launch = gamepad2.right_trigger;
            double rightServoPosition = gamepad2.left_trigger;
            double leftServoPosition = gamepad2.left_trigger;

            // Calculate motor powers
            double leftPower = drive + turn;
            double rightPower = drive - turn;
            double launchPower = launch;
            double rightServoPower = rightServoPosition;
            double leftServoPower = leftServoPosition;

            // Set motor powers
            leftFrontDrive.setPower(0.5*leftPower);
            leftBackDrive.setPower(0.5*leftPower);
            rightFrontDrive.setPower(0.5*rightPower);
            rightBackDrive.setPower(0.5*rightPower);
            launchMotor.setPower(0.42*launchPower);
            rightServo.setPosition(rightServoPower);
            leftServo.setPosition(leftServoPower);

            // Update telemetry
            telemetry.addData("Status", "Running");
            telemetry.addData("Left Power", leftPower);
            telemetry.addData("Right Power", rightPower);
            telemetry.addData("Launch Power", launchPower);
            telemetry.addData("Right Servo Position", rightServoPosition);
            telemetry.addData("Left Servo Position", leftServoPosition);
            telemetry.update();
        }
    }
}
