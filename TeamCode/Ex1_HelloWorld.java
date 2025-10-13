package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "HelloWorld")
public class Ex1_HelloWorld extends OpMode {

    @Override
    public void init() {
        telemetry.addData("Hello","Hello, World");
    }

    @Override
    public void loop() {

    }
}
