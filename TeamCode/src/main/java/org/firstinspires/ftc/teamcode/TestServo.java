package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Test Servo")
public class TestServo extends OpMode {

    Servo servo;

    int position = 0;


    @Override
    public void init() {
        servo = hardwareMap.servo.get("servo");
    }

    @Override
    public void loop() {
        if (gamepad1.y)
            position += 0.01;
        else if (gamepad1.a)
            position -= 0.01;

        if (position > 1)
            position = 1;
        else if (position < 0)
            position = 0;
        servo.setPosition(position);
    }
}
