package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "ManualControl")
public class ManualControl extends OpMode {
    Robot robot = new Robot();
    double EPS = 0.1;
    double ACC = 0.1;
    double currentPowerRunning;
    double currentPowerRotation;
    boolean armButtonPressed = false;
    double[] armPositions = {0, 0.5, 1};
    int currentArmPosition = 0;

    @Override
    public void init() {
        robot.flMotor = hardwareMap.dcMotor.get("fl");
        robot.frMotor = hardwareMap.dcMotor.get("fr");
        robot.blMotor = hardwareMap.dcMotor.get("bl");
        robot.brMotor = hardwareMap.dcMotor.get("br");

        robot.frMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.brMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        robot.armMotor = hardwareMap.dcMotor.get("armMotor");
        robot.armServo = hardwareMap.servo.get("armServo");
        robot.intake = hardwareMap.dcMotor.get("intake");

        robot.armServo.setPosition(0);
    }

    void driveWheels() {
        if (gamepad1.right_bumper || gamepad1.left_bumper) {
            currentPowerRunning = 0;

            if (currentPowerRotation == 0)
                currentPowerRotation = 0.1;
            else if (currentPowerRotation < 1)
                currentPowerRunning = Math.min(currentPowerRunning + ACC, 1);

            if (gamepad1.right_bumper)
                robot.drive(-1, 1, -1, 1, currentPowerRotation);
            else
                robot.drive(1, -1, 1, -1, currentPowerRotation);
        } else {
            currentPowerRotation = 0;

            double xvalue = gamepad1.left_stick_x, yvalue = -gamepad1.left_stick_y;
            double angle = Math.toDegrees(Math.atan2(yvalue, xvalue));
            double power = Math.sqrt(xvalue * xvalue + yvalue * yvalue);

            if (power < EPS)
                currentPowerRunning = 0;
            else if (currentPowerRunning < power)
                currentPowerRunning = Math.min(currentPowerRunning + ACC, power);
            else
                currentPowerRunning = power;

            if (EPS < yvalue) {
                if (angle < 90) {
                    double Dial1WheelsPower = robot.map(angle, 0, 90, -1, 1);
                    if (Dial1WheelsPower < EPS)
                        Dial1WheelsPower = 0;
                    robot.drive(1, Dial1WheelsPower, Dial1WheelsPower, 1, currentPowerRunning);
                } else {
                    double Dial2WheelsPower = robot.map(angle, 90, 180, 1, -1);
                    if (Dial2WheelsPower < EPS)
                        Dial2WheelsPower = 0;
                    robot.drive(Dial2WheelsPower, 1, 1, Dial2WheelsPower, currentPowerRunning);
                }
            } else if (yvalue < -EPS) {
                if (angle < -90) {
                    double Dial3WheelsPower = robot.map(angle, -180, -90, 1, -1);
                    if (Dial3WheelsPower > -EPS)
                        Dial3WheelsPower = 0;
                    robot.drive(-1, Dial3WheelsPower, Dial3WheelsPower, -1, currentPowerRunning);
                } else {
                    double Dial4WheelsPower = robot.map(angle, -90, 0, -1, 1);
                    if (Dial4WheelsPower > -EPS)
                        Dial4WheelsPower = 0;
                    robot.drive(Dial4WheelsPower, -1, -1, Dial4WheelsPower, currentPowerRunning);
                }
            } else {
                if (-EPS < xvalue && xvalue < EPS)
                    robot.stopMotors();
                else if (xvalue > EPS)
                    robot.drive(1, -1, -1, 1, currentPowerRunning);
                else
                    robot.drive(-1, 1, 1, -1, currentPowerRunning);
            }
        }
    }

    void controlArm() {
        if (gamepad1.dpad_up)
            robot.setArmPower(-0.5);
        else if (gamepad1.dpad_down)
            robot.setArmPower(0.5);
        else
            robot.setArmPower(0);

        if (gamepad1.a) {
            if (!armButtonPressed) {
                armButtonPressed = true;

                currentArmPosition++;
                if (currentArmPosition >= 3)
                    currentArmPosition = 0;

                robot.armServo.setPosition(armPositions[currentArmPosition]);
            }
        } else
            armButtonPressed = false;
    }

    void controlIntake() {
        if (gamepad1.x)
            robot.setIntakePower(0.5);
        else
            robot.setIntakePower(0);
    }

    void controlSquishy() {
        if (gamepad1.right_trigger > EPS)
            robot.setSquishyPower(-0.5);
        else if (gamepad1.left_trigger > EPS)
            robot.setSquishyPower(0.5);
        else
            robot.setSquishyPower(0);
    }

    @Override
    public void loop() {
        driveWheels();
        controlArm();
        controlIntake();
        controlSquishy();
    }
}
