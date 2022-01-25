package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "Autonimie Encoder", group = "Encoder")
public class AutonomieEncoder extends LinearOpMode {
    final double PIVOT_SPEED = 0.6;
    final double PIVOT_SLOW_SPEED = 0.35;
    final int COUNTS_PER_INCH = 38;
    final int TICKS_ERROR = 30;
    double speed = 0.60;
    int wheelsReached = 0;


    Robot Robot = new Robot();
    BNO055IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardwareMap();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);

        telemetry.addData("Initialize status: ", "Complete");
        telemetry.addData("IMU Angle: ", getZAngle());
        telemetry.update();

        waitForStart();

        if(!opModeIsActive())
            return;

        goToPosition(-1, 27*COUNTS_PER_INCH, 0.4);
        sleep(1000);
        if(!opModeIsActive())
            return;
    }

    void translateToPosition(int bl, int br, int fl, int fr, int ticks, double robotPower) {
        runToPositionMode();

        Robot.flMotor.setTargetPosition(Robot.flMotor.getCurrentPosition() + fl * ticks);
        Robot.frMotor.setTargetPosition(Robot.frMotor.getCurrentPosition() + fr * ticks);
        Robot.blMotor.setTargetPosition(Robot.blMotor.getCurrentPosition() + bl * ticks);
        Robot.brMotor.setTargetPosition(Robot.brMotor.getCurrentPosition() + br * ticks);
        wheelsReached = 0;
        // telemetry.addData("target position: ", ticks);
        Robot.drive(bl, br, fl, fr, 0.3);

        boolean ok0 = false, ok1 = false, ok2 = false, ok3 = false;

        while (opModeIsActive() && wheelsReached < 4) {

            telemetry.addData("target position: ", ticks);
            telemetry.addData("current position Bleft: ", Robot.blMotor.getCurrentPosition());
            telemetry.addData("current position Fleft: ", Robot.flMotor.getCurrentPosition());
            telemetry.addData("current position Bright: ", Robot.brMotor.getCurrentPosition());
            telemetry.addData("current position Fright: ", Robot.frMotor.getCurrentPosition());
            telemetry.addData("Still working: ", 1);
            telemetry.update();

            if (Math.abs(ticks * bl - Robot.blMotor.getCurrentPosition()) < TICKS_ERROR) {
                Robot.blMotor.setPower(0);
                if (!ok0) {
                    wheelsReached++;
                    ok0 = true;
                }
            }
            if (Math.abs(ticks * br - Robot.brMotor.getCurrentPosition()) < TICKS_ERROR) {
                Robot.brMotor.setPower(0);
                if (!ok1) {
                    wheelsReached++;
                    ok1 = true;
                }
            }
            if (Math.abs(ticks * fl - Robot.flMotor.getCurrentPosition()) < TICKS_ERROR) {
                Robot.flMotor.setPower(0);
                if (!ok2) {
                    wheelsReached++;
                    ok2 = true;
                }
            }
            if (Math.abs(ticks * fr - Robot.frMotor.getCurrentPosition()) < TICKS_ERROR) {
                Robot.frMotor.setPower(0);
                if (!ok3) {
                    wheelsReached++;
                    ok3 = true;
                }
            }
        }
        telemetry.addData("Finished: ", 1);
        telemetry.update();
        Robot.stopMotors();

    }


    void goToPosition(int forwardB, int ticks, double robotPower) {

        runToPositionMode();

        Robot.flMotor.setTargetPosition(Robot.flMotor.getCurrentPosition() + forwardB * ticks);
        Robot.frMotor.setTargetPosition(Robot.frMotor.getCurrentPosition() + forwardB * ticks);
        Robot.blMotor.setTargetPosition(Robot.blMotor.getCurrentPosition() + forwardB * ticks);
        Robot.brMotor.setTargetPosition(Robot.brMotor.getCurrentPosition() + forwardB * ticks);
        wheelsReached = 0;

        Robot.drive(forwardB, forwardB, forwardB, forwardB, 0.3);

        boolean ok0 = false, ok1 = false, ok2 = false, ok3 = false;

        while (opModeIsActive() && wheelsReached < 4) {

            telemetry.addData("target position: ", ticks);
            telemetry.addData("current position Fleft: ", Robot.flMotor.getCurrentPosition());
            telemetry.addData("current position Fright: ", Robot.frMotor.getCurrentPosition());
            telemetry.addData("current position Bleft: ", Robot.blMotor.getCurrentPosition());
            telemetry.addData("current position Bright: ", Robot.brMotor.getCurrentPosition());
            telemetry.addData("Still working: ", 1);
            telemetry.update();

            if (Math.abs(ticks * forwardB - Robot.blMotor.getCurrentPosition()) < TICKS_ERROR) {
                Robot.blMotor.setPower(0);
                if (!ok0) {
                    wheelsReached++;
                    ok0 = true;
                }
            }
            if (Math.abs(ticks * forwardB - Robot.brMotor.getCurrentPosition()) < TICKS_ERROR) {
                Robot.brMotor.setPower(0);
                if (!ok1) {
                    wheelsReached++;
                    ok1 = true;
                }
            }
            if (Math.abs(ticks * forwardB - Robot.flMotor.getCurrentPosition()) < TICKS_ERROR) {
                Robot.flMotor.setPower(0);
                if (!ok2) {
                    wheelsReached++;
                    ok2 = true;
                }
            }
            if (Math.abs(ticks * forwardB - Robot.frMotor.getCurrentPosition()) < TICKS_ERROR) {
                Robot.frMotor.setPower(0);
                if (!ok3) {
                    wheelsReached++;
                    ok3 = true;
                }
            }
        }
        telemetry.addData("Finished: ", 1);
        telemetry.update();
        Robot.stopMotors();
    }

    void rotateRight(double targetAngle) {
        runByPowerMode();

        telemetry.addData("Started rotating: ", targetAngle);
        telemetry.update();

        while (getZAngle() < targetAngle && opModeIsActive()) {
            if (getZAngle() < 2.0 / 3 * targetAngle) {
                Robot.drive(1, -1, 1, -1, PIVOT_SPEED);
            } else {
                Robot.drive(1, -1, 1, -1, PIVOT_SPEED / 2);
            }

            telemetry.addData("IMU Angle", getZAngle());
            telemetry.update();
        }
        Robot.stopMotors();
    }

    void rotateLeft(double targetAngle) {
        runByPowerMode();

        telemetry.addData("Started rotating: ", targetAngle);
        telemetry.update();

        while (getZAngle() > targetAngle && opModeIsActive()) {
            if (getZAngle() > targetAngle + 15) {
                Robot.drive(-1, 1, -1, 1, PIVOT_SPEED);
            } else {
                Robot.drive(-1, 1, -1, 1, PIVOT_SPEED / 2);
            }

            telemetry.addData("IMU Angle: ", getZAngle());
            telemetry.update();
        }
        Robot.stopMotors();
    }

    void initHardwareMap() {
        Robot.flMotor = hardwareMap.dcMotor.get("fl");
        Robot.frMotor = hardwareMap.dcMotor.get("fr");
        Robot.blMotor = hardwareMap.dcMotor.get("bl");
        Robot.brMotor = hardwareMap.dcMotor.get("br");

        Robot.flMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        Robot.blMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        Robot.frMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Robot.brMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Robot.flMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Robot.blMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Robot.frMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Robot.brMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Robot.flMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Robot.blMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Robot.frMotor.setTargetPosition(0);
        Robot.brMotor.setTargetPosition(0);
        Robot.flMotor.setTargetPosition(0);
        Robot.blMotor.setTargetPosition(0);

        Robot.frMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Robot.brMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Robot.flMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Robot.blMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Robot.frMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Robot.brMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Robot.flMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Robot.blMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Hardware Map Init Complete");
        telemetry.update();
    }

    void runByPowerMode() {
        Robot.frMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Robot.brMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Robot.flMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Robot.blMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    void runToPositionMode() {

        Robot.frMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Robot.brMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Robot.flMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Robot.blMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Robot.frMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Robot.brMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Robot.flMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Robot.blMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Robot.frMotor.setTargetPosition(0);
        Robot.brMotor.setTargetPosition(0);
        Robot.flMotor.setTargetPosition(0);
        Robot.blMotor.setTargetPosition(0);

        Robot.frMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Robot.brMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Robot.flMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Robot.blMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private double getZAngle() {
        return (-imu.getAngularOrientation().firstAngle);
    }
}
