package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.robot.Robot;
import org.firstinspires.ftc.teamcode.RobotTest;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.StrictMath.max;

@Autonomous(name = "Autonomie Timer Blue Cube Only Storage Unit")
public class AutonomieTimerBlueCubeOnlyStorageUnit extends LinearOpMode {
    RobotTest Robot = new RobotTest();
    ElapsedTime timer = new ElapsedTime();
    double acc = 0.55;
    double speed = 0.80;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardwareMap();
        waitForStart();

        DriveAcc(1, 1, 1, 1, speed, 1.26, acc, 0.02);

        if (!opModeIsActive())
            return;

        Robot.setArmPower(-0.6);

        timer.reset();
        while (timer.seconds() < 3.5 && opModeIsActive()) ;
        Robot.setArmPower(0);
        if (!opModeIsActive())
            return;

        Robot.armServo.setPosition(1);

        while (timer.seconds() < 5 && opModeIsActive()) ;

        Robot.armServo.setPosition(0);

        Robot.setArmPower(0.6);
        timer.reset();
        while (timer.seconds() < 1.9 && opModeIsActive()) ;
        Robot.setArmPower(0);
        if (!opModeIsActive())
            return;

        timer.reset();
        while (timer.seconds() < 1 && opModeIsActive()) ;
        if (!opModeIsActive())
            return;

        DriveAcc(1, -1, 1, -1, speed, 0.7, acc, 0.02);

        timer.reset();
        while (timer.seconds() < 0.5 && opModeIsActive()) ;
        if (!opModeIsActive())
            return;

        DriveAcc(1, 1, 1, 1, speed, 0.92, acc, 0.02);

        timer.reset();
        while (timer.seconds() < 0.5 && opModeIsActive()) ;
        if (!opModeIsActive())
            return;

        DriveAcc(1, -1, 1, -1, speed, 0.62, acc, 0.02);

        timer.reset();
        while (timer.seconds() < 0.5 && opModeIsActive()) ;
        if (!opModeIsActive())
            return;

        DriveAcc(1, 1, 1, 1, speed, 0.3, acc, 0.02);
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

        Robot.armServo.setPosition(0);

        telemetry.addData("Status", "Hardware Map Init Complete");
        telemetry.update();
    }
}
