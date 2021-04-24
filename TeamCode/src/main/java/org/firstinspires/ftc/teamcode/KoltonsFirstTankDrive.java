package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.Rotation;
import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;

@TeleOp(name = "KoltonsFirstTankDrive", group = "ForRobot")

public class KoltonsFirstTankDrive extends LinearOpMode {
    RobotHardware robot = new RobotHardware();
    RobotFunctions functions = new RobotFunctions();

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        double DriveTicksPerRotation = 28;
        double DriveMaxTicks = (DriveTicksPerRotation*100);
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                double GamepadLeft = (-gamepad1.left_stick_y);
                double GamepadRight = (-gamepad1.right_stick_y);
                robot.LeftDrive.setVelocity(GamepadLeft*DriveMaxTicks);
                robot.RightDrive.setVelocity(GamepadRight*DriveMaxTicks);
                functions.Telementrys();

                if (gamepad2.a);
                robot.LeftShooter.setVelocity(0);
                robot.RightShooter.setVelocity(0);

            }
        }
    }
}