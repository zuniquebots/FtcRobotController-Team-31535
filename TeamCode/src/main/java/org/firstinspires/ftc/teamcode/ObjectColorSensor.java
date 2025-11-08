package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

public class ObjectColorSensor extends LinearOpMode {
    public static boolean isGreenOrPurple(int R, int G, int B) {
        boolean isGreen = G > R * 1.5 && G > B * 1.5 && G > 50; // Green detection
        boolean isPurple = R > G * 1.5 && B > G * 1.5 && R + B > 150; // Purple detection
        // True if both green and purple are detected;
        return isGreen || isPurple;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        while (opModeIsActive()) {
            // Example RGB values
            int R = 120, G = 200, B = 80; // Sample it's for green ball
            int R2 = 180, G2 = 50, B2 = 200; // Sample it's for purple ball

            // Check if one of the colors colors are detected
            boolean result = isGreenOrPurple(R, G, B) || isGreenOrPurple(R2, G2, B2);
            if (result) {
                telemetry.addData("Ball detected:", true);
            } else {
                telemetry.addData("Ball detected:", false);
            }
            telemetry.update();
        }
    }
}