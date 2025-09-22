package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@TeleOp(name="Minimal TFOD Logitech", group="Test")
public class MinimalTFODLogitech extends LinearOpMode {

    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {

        // 1️⃣ Get the Logitech webcam from hardware map
        WebcamName webcam = hardwareMap.get(WebcamName.class, "Webcam 1");

        if (webcam == null) {
            telemetry.addData("Camera", "Not Found!");
            telemetry.update();
            return; // Stop if camera is not found
        } else {
            telemetry.addData("Camera", "Found!");
            telemetry.update();
        }

        // 2️⃣ Initialize TensorFlow Object Detection
        int tfodMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);

        tfod = ClassFactory.getInstance().getTFObjectDetector(tfodParameters, webcam);

        // Load your model (must be in app/src/main/assets)
        tfod.loadModelFromAsset("FreightFrenzy_BCDM.tflite", "Ball", "Cube", "Duck");

        // 3️⃣ Wait for start
        telemetry.addData("Status", "Press play to start");
        telemetry.update();
        waitForStart();

        // 4️⃣ Activate TFOD
        if (tfod != null) tfod.activate();

        // 5️⃣ Main loop
        while (opModeIsActive()) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

            if (updatedRecognitions != null) {
                telemetry.addData("# Objects Detected", updatedRecognitions.size());
                for (Recognition recognition : updatedRecognitions) {
                    telemetry.addData("Object", recognition.getLabel() +
                            " | Confidence: " + String.format("%.2f", recognition.getConfidence()));
                }
            }

            telemetry.update();
        }

        // 6️⃣ Shutdown TFOD
        if (tfod != null) tfod.shutdown();
    }

    private class TFObjectDetector {
    }
}
