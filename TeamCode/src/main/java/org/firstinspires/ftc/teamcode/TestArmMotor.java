package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@TeleOp(name = "Test Arm Motor")
public class TestArmMotor extends OpMode{
    RobotTest robot = new RobotTest();
    int targetPosition = 0;

    @Override
    public void init() {
        robot.armMotor = hardwareMap.dcMotor.get("armMotor");
        robot.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.armMotor.setTargetPosition(0);
    }

    @Override
    public void loop() {
        if(gamepad1.dpad_up)
            targetPosition = 0;
        else if(gamepad1.dpad_down)
            targetPosition = 100;

        robot.armMotor.setTargetPosition(targetPosition);

        robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if(robot.armMotor.getCurrentPosition()<targetPosition)
            robot.armMotor.setPower(1);
        else
            robot.armMotor.setPower(1);

        telemetry.addData("Current position:", robot.armMotor.getCurrentPosition());
    }
}