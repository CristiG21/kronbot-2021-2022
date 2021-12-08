package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.Date;

@TeleOp(name = "Wheels_Arm")
public class Wheels_Arm extends OpMode {
    Robot robot = new Robot();
    double epsm = 0.1;
    int currentServoPosition = 0;
    double[] servoPositions = {0, 0.5, 1};

    @Override
    public void init() {
        robot.flMotor = hardwareMap.dcMotor.get("fl");
        robot.frMotor = hardwareMap.dcMotor.get("fr");
        robot.blMotor = hardwareMap.dcMotor.get("bl");
        robot.brMotor = hardwareMap.dcMotor.get("br");

        robot.flMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.blMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        robot.armMotor = hardwareMap.dcMotor.get("armMotor");
        robot.armServo = hardwareMap.servo.get("armServo");
        robot.broom = hardwareMap.dcMotor.get("broom");

        robot.armServo.setPosition(0);
    }

    void driveWheels() {
        if (gamepad1.right_bumper)
            robot.powerWheelsMotors(-1, 1, -1, 1, 0.5);
        else if (gamepad1.left_bumper)
            robot.powerWheelsMotors(1, -1, 1, -1, 0.5);
        else {
            double xvalue = gamepad1.left_stick_x, yvalue = -gamepad1.left_stick_y;
            double angle = Math.toDegrees(Math.atan2(yvalue, xvalue));

            double power = Math.sqrt(xvalue * xvalue + yvalue * yvalue);
            if (power < epsm)
                power = 0;

            if (epsm < yvalue) {
                if (angle < 90) {
                    double Dial1WheelsPower = map(angle, 0, 90, -1, 1);
                    if (Dial1WheelsPower < epsm)
                        Dial1WheelsPower = 0;
                    robot.powerWheelsMotors(1, Dial1WheelsPower, Dial1WheelsPower, 1, power);
                } else {
                    double Dial2WheelsPower = map(angle, 90, 180, 1, -1);
                    if (Dial2WheelsPower < epsm)
                        Dial2WheelsPower = 0;
                    robot.powerWheelsMotors(Dial2WheelsPower, 1, 1, Dial2WheelsPower, power);
                }
            } else if (yvalue < -epsm) {
                if (angle < -90) {
                    double Dial3WheelsPower = map(angle, -180, -90, 1, -1);
                    if (Dial3WheelsPower > -epsm)
                        Dial3WheelsPower = 0;
                    robot.powerWheelsMotors(-1, Dial3WheelsPower, Dial3WheelsPower, -1, power);
                } else {
                    double Dial4WheelsPower = map(angle, -90, 0, -1, 1);
                    if (Dial4WheelsPower > -epsm)
                        Dial4WheelsPower = 0;
                    robot.powerWheelsMotors(Dial4WheelsPower, -1, -1, Dial4WheelsPower, power);
                }
            } else {
                if (-epsm < xvalue && xvalue < epsm)
                    robot.stopMotor();
                else if (xvalue > epsm)
                    robot.powerWheelsMotors(1, -1, -1, 1, power);
                else
                    robot.powerWheelsMotors(-1, 1, 1, -1, power);
            }
        }

        if (gamepad1.dpad_up)
            robot.powerArmMotor(0.5);
        else if (gamepad1.dpad_down)
            robot.powerArmMotor(-0.5);
    }

    void controlArm() {
        if (gamepad1.dpad_up)
            robot.powerArmMotor(-0.5);
        else if (gamepad1.dpad_down)
            robot.powerArmMotor(0.5);
        else
            robot.powerArmMotor(0);

        if (gamepad1.a) {
            currentServoPosition++;
            if (currentServoPosition == 3)
                currentServoPosition = 0;
            robot.armServo.setPosition(servoPositions[currentServoPosition]);
        }
    }

    void controlBroom() {
        if (gamepad1.right_trigger > epsm)
            robot.powerBroom(0.5);
        else
            robot.powerBroom(0);
    }

    double map(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) / (in_max - in_min) * (out_max - out_min) + out_min;
    }

    @Override
    public void loop() {
        driveWheels();
        controlArm();
        controlBroom();
    }
}
