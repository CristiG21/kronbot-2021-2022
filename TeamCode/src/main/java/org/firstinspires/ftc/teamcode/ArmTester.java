package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "ArmTester")
public class ArmTester extends OpMode {
    Robot robot = new Robot();

    @Override
    public void init() {
        robot.armMotor = hardwareMap.dcMotor.get("armMotor");
        robot.armServo = hardwareMap.servo.get("armServo");
    }

    void drive() {
        if (gamepad1.dpad_up)
            robot.powerArmMotor(0.5);
        else if (gamepad1.dpad_down)
            robot.powerArmMotor(-0.5);
        else
            robot.powerArmMotor(0);

        if (gamepad1.dpad_right)
            robot.armServo.setPosition(1);
        else if (gamepad1.dpad_left)
            robot.armServo.setPosition(0);
    }

    @Override
    public void loop() {
        drive();
    }
}
