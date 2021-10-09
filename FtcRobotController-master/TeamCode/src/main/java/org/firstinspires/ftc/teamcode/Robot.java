package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Robot {
    DcMotor flMotor;
    DcMotor frMotor;
    DcMotor blMotor;
    DcMotor brMotor;

    void powerMotor(double frontLeft, double frontRight, double backLeft, double backRight, double power) {
        flMotor.setPower(frontLeft * power);
        frMotor.setPower(frontRight * power);
        blMotor.setPower(backLeft * power);
        brMotor.setPower(backRight * power);
    }

    void stopMotor() {
        flMotor.setPower(0);
        frMotor.setPower(0);
        blMotor.setPower(0);
        brMotor.setPower(0);
    }
}
