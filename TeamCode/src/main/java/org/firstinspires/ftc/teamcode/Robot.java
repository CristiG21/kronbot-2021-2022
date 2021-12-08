package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Robot {
    DcMotor flMotor;
    DcMotor frMotor;
    DcMotor blMotor;
    DcMotor brMotor;

    DcMotor armMotor;
    Servo armServo;
    DcMotor broom;

    void powerWheelsMotors(double frontLeft, double frontRight, double backLeft, double backRight, double power) {
        flMotor.setPower(frontLeft * power);
        frMotor.setPower(frontRight * power);
        blMotor.setPower(backLeft * power);
        brMotor.setPower(backRight * power);
    }

    void powerArmMotor(double power){
        armMotor.setPower(power);
    }

    void powerBroom(double power){
        broom.setPower(power);
    }

    void stopMotor() {
        flMotor.setPower(0);
        frMotor.setPower(0);
        blMotor.setPower(0);
        brMotor.setPower(0);
    }
}
