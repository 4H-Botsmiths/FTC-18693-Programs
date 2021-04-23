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


        robot.LeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.RightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        double DriveTicksPerRotation = 28;
        double DriveMaxTicks = (DriveTicksPerRotation*100);
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                double GamepadLeft = (-gamepad1.left_stick_y);
                double GamepadRight = (-gamepad1.right_stick_y);
                robot.LeftDrive.setVelocity(GamepadLeft*DriveMaxTicks);
                robot.RightDrive.setVelocity(GamepadRight*DriveMaxTicks);
                robot.LeftShooter.setVelocity(0);
                robot.RightShooter.setVelocity(0);

            }
        }
    }
}