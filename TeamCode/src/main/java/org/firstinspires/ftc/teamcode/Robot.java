package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Robot {
    DcMotor flMotor;
    DcMotor frMotor;
    DcMotor blMotor;
    DcMotor brMotor;

    DcMotor armMotor;
    Servo armServo;

    DcMotor intake;

    DcMotor squishy;

    /**
     * Set the power of the motors from wheels
     *
     * @param frontLeft  direction of the front left wheel -1 or 1
     * @param frontRight direction of the front right wheel -1 or 1
     * @param backLeft   direction of the back left wheel -1 or 1
     * @param backRight  direction of the back right wheel -1 or 1
     * @param power      the power to give to all four wheels [0,1]
     */
    void drive(double frontLeft, double frontRight, double backLeft, double backRight, double power) {
        flMotor.setPower(frontLeft * power);
        frMotor.setPower(frontRight * power);
        blMotor.setPower(backLeft * power);
        brMotor.setPower(backRight * power);
    }

    /**
     * Stops the motors from wheels
     */
    void stopMotors() {
        flMotor.setPower(0);
        frMotor.setPower(0);
        blMotor.setPower(0);
        brMotor.setPower(0);
    }

    /**
     * Set th power of the arm motor
     *
     * @param power the power to give to the motor [0,1]
     */
    void setArmPower(double power) {
        armMotor.setPower(power);
    }

    /**
     * Set the power of the intake motor
     *
     * @param power the power to give to the motor [0,1]
     */
    void setIntakePower(double power) {
        intake.setPower(power);
    }

    /**
     * Set the power of the squishy motor
     *
     * @param power the power to give to the motor [0,1]
     */
    void setSquishyPower(double power) {
        squishy.setPower(power);
    }

    double map(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) / (in_max - in_min) * (out_max - out_min) + out_min;
    }
}
