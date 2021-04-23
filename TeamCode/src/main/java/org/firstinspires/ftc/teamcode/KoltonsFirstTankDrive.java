package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

@TeleOp(name = "KoltonsFirstTankDrive", group = "ForRobot")

public class KoltonsFirstTankDrive extends LinearOpMode {
    RobotHardware robot = new RobotHardware();

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);


        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        double MotorTicks = 28;
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                double MotorLeft = Math.pow(-gamepad1.left_stick_y, 3) * MotorTicks;
                double MotorRight = Math.pow(-gamepad1.right_stick_y, 3) * MotorTicks;
                robot.leftDrive.setVelocity(MotorLeft);
                robot.rightDrive.setVelocity(MotorRight);

                double MotorLeft = Math.pow(-gamepad1.left_stick_y, 3);
                double MotorRight = Math.pow(-gamepad1.right_stick_y, 3);
                robot.leftDrive.setPower(MotorLeft);
                robot.rightDrive.setPower(MotorRight);
                robot.leftShooter.setVelocity();
                robot.rightShooter.setVelocity();

            }
        }
    }
}