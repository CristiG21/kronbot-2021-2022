package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "ExempluAutonomie")
public class ExempluAutonomie extends LinearOpMode {
    Robot robot = new Robot();
    int eps = 30;
    BNO055IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.flMotor = hardwareMap.dcMotor.get("fl");
        robot.frMotor = hardwareMap.dcMotor.get("fr");
        robot.blMotor = hardwareMap.dcMotor.get("bl");
        robot.brMotor = hardwareMap.dcMotor.get("br");

        robot.flMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.blMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        robot.flMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.flMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.flMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.flMotor.setTargetPosition(0);

        robot.frMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.frMotor.setTargetPosition(0);

        robot.blMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.blMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.blMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.blMotor.setTargetPosition(0);

        robot.brMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.brMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.brMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.brMotor.setTargetPosition(0);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);

        waitForStart();
    }

    void RunToPosition(int sign, int ticks, double power) {
        int[] target = {robot.flMotor.getCurrentPosition() + sign * ticks, robot.frMotor.getCurrentPosition() + sign * ticks, robot.blMotor.getCurrentPosition() + sign * ticks, robot.brMotor.getCurrentPosition() + sign * ticks};

        robot.flMotor.setTargetPosition(target[0]);
        robot.frMotor.setTargetPosition(target[1]);
        robot.blMotor.setTargetPosition(target[2]);
        robot.brMotor.setTargetPosition(target[3]);
        robot.powerMotor(sign, sign, sign, sign, power);

        int ok = 0;

        while (opModeIsActive() && ok < 4) {
            ok = 0;
            if (target[0] - robot.flMotor.getCurrentPosition() <= eps) {
                ok++;
                robot.flMotor.setPower(0);
            }
            if (target[1] - robot.frMotor.getCurrentPosition() <= eps) {
                ok++;
                robot.frMotor.setPower(0);
            }
            if (target[2] - robot.blMotor.getCurrentPosition() <= eps) {
                ok++;
                robot.blMotor.setPower(0);
            }
            if (target[3] - robot.brMotor.getCurrentPosition() <= eps) {
                ok++;
                robot.brMotor.setPower(0);
            }
        }
    }

    private double getZAngle() {
        return (-imu.getAngularOrientation().firstAngle);
    }

    void RotateRight(double angle) {
        robot.flMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.blMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.brMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeIsActive() && getZAngle() < angle) {
            if (angle - getZAngle() <= eps)
                robot.powerMotor(-1, 1, -1, 1, 0.3);
            else
                robot.powerMotor(-1, 1, -1, 1, 0.6);
        }
        robot.stopMotor();
    }

    void RotateLeft(double angle) {
        robot.flMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.blMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.brMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeIsActive() && getZAngle() > angle) {
            if (getZAngle() - angle <= eps)
                robot.powerMotor(1, -1, 1, -1, 0.3);
            else
                robot.powerMotor(1, -1, 1, -1, 0.6);
        }
        robot.stopMotor();
    }
}
