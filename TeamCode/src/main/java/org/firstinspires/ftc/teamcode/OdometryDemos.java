package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "Mecanum Pinpoint TurnToHeading", group = "Examples")
public class OdometryDemos extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize motors (make sure names match your config)
        frontLeft = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        frontRight = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        backLeft = hardwareMap.get(DcMotor.class, "leftBackDrive");
        backRight = hardwareMap.get(DcMotor.class, "rightBackDrive");

        // Reverse the correct motors (usually right side)
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        // Initialize Pinpoint
        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "PinpointComputer");

        telemetry.addLine("Calibrating Pinpoint... keep robot still");
        telemetry.update();
        pinpoint.resetPosAndIMU();  // resets both IMU and odometry
        sleep(1000);
        telemetry.addLine("Calibration complete!");
        telemetry.update();

        waitForStart();

        // Example: turn to 90 degrees absolute heading
        double targetHeading = 90.0;

        // PID constants — tune for your robot
        double kP = 0.02;
        double kI = 0.0;
        double kD = 0.002;

        double integral = 0;
        double lastError = 0;

        while (opModeIsActive()) {
            pinpoint.update();

            Pose2D pose = pinpoint.getPosition();
            double currentHeading = Math.toDegrees(pose.getHeading(AngleUnit.DEGREES)); // Corrected: Get heading in degrees

            // Compute shortest signed error (-180..180)
            double errorRad = Math.atan2(
                    Math.sin(Math.toRadians(targetHeading - currentHeading)),
                    Math.cos(Math.toRadians(targetHeading - currentHeading))
            );
            double errorDeg = Math.toDegrees(errorRad);

            // PID math
            integral += errorDeg;
            double derivative = errorDeg - lastError;
            lastError = errorDeg;

            double turnPower = kP * errorDeg + kI * integral + kD * derivative;
            turnPower = Math.max(-1.0, Math.min(1.0, turnPower)); // clamp

            // Apply rotational power to all 4 mecanum wheels equally
            frontLeft.setPower(turnPower);
            backLeft.setPower(turnPower);
            frontRight.setPower(-turnPower);
            backRight.setPower(-turnPower);

            telemetry.addData("Target Heading", targetHeading);
            telemetry.addData("Current Heading", currentHeading);
            telemetry.addData("Error (deg)", errorDeg);
            telemetry.addData("Turn Power", turnPower);
            telemetry.update();

            // stop when within 2 degrees
            if (Math.abs(errorDeg) < 2) {
                stopAllMotors();
                telemetry.addLine("Target reached!");
                telemetry.update();
                break;
            }
        }
    }

    private void stopAllMotors() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}
