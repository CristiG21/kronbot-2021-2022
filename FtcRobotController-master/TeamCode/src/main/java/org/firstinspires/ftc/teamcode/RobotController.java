package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "ManualCtrl")
public class RobotController extends OpMode {
    Robot robot = new Robot();
    double epsm = 0.1;
    double a = 0.8;

    @Override
    public void init() {
        robot.flMotor = hardwareMap.dcMotor.get("fl");
        robot.frMotor = hardwareMap.dcMotor.get("fr");
        robot.blMotor = hardwareMap.dcMotor.get("bl");
        robot.brMotor = hardwareMap.dcMotor.get("br");

        robot.flMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.blMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    void drive() {
        if (gamepad1.right_bumper)
            robot.powerMotor(-1, 1, -1, 1, 0.5);
        else if (gamepad1.left_bumper)
            robot.powerMotor(1, -1, 1, -1, 0.5);
        else {
            double xvalue = gamepad1.left_stick_x, yvalue = -gamepad1.left_stick_y;
            double angle = Math.toDegrees(Math.atan2(yvalue, xvalue));

            if (epsm < yvalue) {
                if (angle < 90) {
                    double RightWheelsPower = map(angle, 0, 90, 0, 1);
                    if (RightWheelsPower < epsm)
                        RightWheelsPower = 0;
                    robot.powerMotor(1, RightWheelsPower, 1, RightWheelsPower, 1);
                } else {
                    double LeftWheelsPower = map(angle, 90, 180, 1, 0);
                    if (LeftWheelsPower < epsm)
                        LeftWheelsPower = 0;
                    robot.powerMotor(LeftWheelsPower, 1, LeftWheelsPower, 1, 1);
                }
            } else if (yvalue < -epsm) {
                if (angle < -90) {
                    double LeftWheelsPower = map(angle, -180, -90, 0, -1);
                    if (LeftWheelsPower > -epsm)
                        LeftWheelsPower = 0;
                    robot.powerMotor(LeftWheelsPower, -1, LeftWheelsPower, -1, 1);
                } else {
                    double RightWheelsPower = map(angle, -90, 0, -1, 0);
                    if (RightWheelsPower > -epsm)
                        RightWheelsPower = 0;
                    robot.powerMotor(-1, RightWheelsPower, -1, RightWheelsPower, 1);
                }
            } else {
                if (-epsm < xvalue && xvalue < epsm)
                    robot.stopMotor();
                else if (xvalue > epsm)
                    robot.powerMotor(1, -1, 1, -1, 1);
                else
                    robot.powerMotor(-1, 1, -1, 1, 1);
            }
        }
    }

    double map(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) / (in_max - in_min) * (out_max - out_min) + out_min;
    }

    @Override
    public void loop() {
        drive();
    }
}
