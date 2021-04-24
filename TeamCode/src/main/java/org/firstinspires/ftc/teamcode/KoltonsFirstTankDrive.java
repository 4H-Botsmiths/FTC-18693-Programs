package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
                robot.leftDrive.setVelocity(GamepadLeft*DriveMaxTicks);
                robot.rightDrive.setVelocity(GamepadRight*DriveMaxTicks);
                functions.Telementrys();

                if (gamepad2.a);
                robot.leftShooter.setVelocity(0);
                robot.rightShooter.setVelocity(0);

            }
        }
    }
}