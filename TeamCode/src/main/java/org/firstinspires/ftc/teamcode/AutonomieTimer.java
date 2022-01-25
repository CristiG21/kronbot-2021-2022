package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.StrictMath.max;

@Autonomous(name = "Autonomie Timer")
public class AutonomieTimer extends LinearOpMode {
    Robot Robot = new Robot();
    ElapsedTime timer = new ElapsedTime();
    double acc = 0.55;
    double speed = 0.80;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardwareMap();
        waitForStart();

        Robot.setArmPower(-0.6);
        timer.reset();
        while (timer.seconds() < 1 && opModeIsActive()) ;
        Robot.setArmPower(0);
        if (!opModeIsActive())
            return;

        DriveAcc(-1, 1, 1, -1, speed, 0.35, acc, 0.02);

        timer.reset();
        while (timer.seconds() < 0.5 && opModeIsActive()) ;
        if (!opModeIsActive())
            return;

        DriveAcc(-1, -1, -1, -1, speed, 0.32, acc, 0.02);

        Robot.setSquishyPower(-0.5);

        timer.reset();
        while (timer.seconds() < 5.5 && opModeIsActive()) ;
        Robot.setSquishyPower(0);
        if (!opModeIsActive())
            return;

        DriveAcc(-1, 1, 1, -1, speed, 1.5, acc, 0.02);

        timer.reset();
        while (timer.seconds() < 0.5 && opModeIsActive()) ;
        if (!opModeIsActive())
            return;

        DriveAcc(1, 1, 1, 1, speed, 1, acc, 0.02);

        Robot.setArmPower(-0.6);
        timer.reset();
        while (timer.seconds() < 2.4 && opModeIsActive()) ;
        Robot.setArmPower(0);
        if (!opModeIsActive())
            return;

        Robot.armServo.setPosition(1);

        while (timer.seconds() < 5 && opModeIsActive()) ;

        Robot.armServo.setPosition(0);

        Robot.setArmPower(0.6);
        timer.reset();
        while (timer.seconds() < 0.8 && opModeIsActive()) ;
        Robot.setArmPower(0);
        if (!opModeIsActive())
            return;

        DriveAcc(1, 1, 1, 1, speed, 3.5, acc, 0.02);
    }

    void DriveAcc(double bl, double br, double fl, double fb, double power, double runTime, double acc, double add) {
        timer.reset();

        while (timer.seconds() < runTime - runTime / 4) {
            acc = max(acc - add, 0);
            Robot.drive(bl, br, fl, fb, power - acc);
            if (!opModeIsActive())
                return;
        }
        while (timer.seconds() < runTime) {
            acc = Math.min(power - 0.2, acc + add);
            Robot.drive(bl, br, fl, fb, power - acc);
            if (!opModeIsActive())
                return;
        }

        Robot.drive(bl, br, fl, fb, 0);
    }

    void initHardwareMap() {
        Robot.flMotor = hardwareMap.dcMotor.get("fl");
        Robot.frMotor = hardwareMap.dcMotor.get("fr");
        Robot.blMotor = hardwareMap.dcMotor.get("bl");
        Robot.brMotor = hardwareMap.dcMotor.get("br");

        Robot.flMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        Robot.blMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        Robot.squishy = hardwareMap.dcMotor.get("squishy");
        Robot.armMotor = hardwareMap.dcMotor.get("armMotor");
        Robot.armServo = hardwareMap.servo.get("armServo");

        telemetry.addData("Status", "Hardware Map Init Complete");
        telemetry.update();
    }
}
