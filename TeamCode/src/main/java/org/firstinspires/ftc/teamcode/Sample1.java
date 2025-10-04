package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "1st TeleOP", group = "Examples")
public class Sample1 extends LinearOpMode {
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftFrontDrive = null;

    @Override
    public void runOpMode() {
        // Initialize hardware
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class,"leftBackDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class,"rightBackDrive");

        // Set motor directions (adjust as needed)
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);



        // Send telemetry data to Driver Station
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Get gamepad input
            double drive = -gamepad2.left_stick_y;
            double strafe = gamepad2.left_stick_x;
            double turn = gamepad2.right_stick_x;

            // Calculate motor powers
            double frontLeftPower  = drive + strafe + turn;
            double frontRightPower = drive - strafe - turn;
            double backLeftPower   = drive - strafe + turn;
            double backRightPower  = drive + strafe - turn;


            // Set motor powers
            leftFrontDrive.setPower(0.5*frontLeftPower);
            rightFrontDrive.setPower(0.5*frontRightPower);
            leftBackDrive.setPower(0.5*backLeftPower);
            rightBackDrive.setPower(0.5*backRightPower);


            // Update telemetry
            telemetry.addData("Status", "Running");
            telemetry.addData("Left Power", frontLeftPower);
            telemetry.addData("Right Power", frontRightPower);
            telemetry.update();
        }
    }
}
