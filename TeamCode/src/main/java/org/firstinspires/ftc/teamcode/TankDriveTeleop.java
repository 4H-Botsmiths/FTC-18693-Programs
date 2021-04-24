package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "TankDriveTeleop", group = "ForRobot")

public class TankDriveTeleop extends LinearOpMode {
    RobotHardware robot = new RobotHardware();
    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        double launcher;
        double pickup = 0;
        double middle = 0;
        double feeder = 0;
        double claw;
        double drawbridge;
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                double LeftY = Math.pow(-gamepad1.left_stick_y, 3);
                double RightY = Math.pow(-gamepad1.right_stick_y, 3);
                if (gamepad1.b) {
                    if (pickup == 0) {
                        if (gamepad1.right_bumper) {
                            robot.rampBottom.setDirection(DcMotorSimple.Direction.REVERSE);
                        } else {
                            robot.rampBottom.setDirection(DcMotorSimple.Direction.FORWARD);
                        }
                        robot.rampBottom.setPower(1);
                        pickup = 1;
                    } else {
                        robot.rampBottom.setPower(0);
                        pickup = 0;
                    }
                    sleep(500);
                }
                if (gamepad1.y) {
                    if (middle == 0) {
                        if (gamepad1.right_bumper) {
                            robot.rampMiddle.setDirection(DcMotorSimple.Direction.REVERSE);
                        } else {
                            robot.rampMiddle.setDirection(DcMotorSimple.Direction.FORWARD);
                        }
                        robot.rampMiddle.setPower(1);
                        middle = 1;
                    } else {
                        robot.rampMiddle.setPower(0);
                        middle = 0;
                    }
                    sleep(500);
                }
                if (gamepad1.x) {
                    if (feeder == 0) {
                        if (gamepad1.right_bumper) {
                            robot.rampTop.setDirection(DcMotorSimple.Direction.REVERSE);
                        } else {
                            robot.rampTop.setDirection(DcMotorSimple.Direction.FORWARD);
                        }
                        robot.rampTop.setPower(1);
                        feeder = 1;
                    } else {
                        robot.rampTop.setPower(0);
                        feeder = 0;
                    }
                    sleep(500);
                }
                if (gamepad1.dpad_up) {
                    if (gamepad1.right_bumper) {
                        robot.clawHand.setDirection(DcMotorSimple.Direction.REVERSE);
                    } else {
                        robot.clawHand.setDirection(DcMotorSimple.Direction.FORWARD);
                    }
                    robot.clawHand.setPower(1);
                    claw = 1;
                } else {
                    robot.clawHand.setPower(0);
                    claw = 0;
                }
                if (gamepad1.dpad_down) {
                    if (gamepad1.right_bumper) {
                        robot.clawArm.setDirection(DcMotorSimple.Direction.REVERSE);
                    } else {
                        robot.clawArm.setDirection(DcMotorSimple.Direction.FORWARD);
                    }
                    robot.clawArm.setPower(1);
                    drawbridge = 1;
                } else {
                    robot.clawArm.setPower(0);
                    drawbridge = 0;
                }
                robot.leftDrive.setPower(LeftY);
                robot.rightDrive.setPower(RightY);
                robot.leftShooter.setVelocity(1820 * gamepad1.left_trigger);
                robot.rightShooter.setVelocity(1820 * gamepad1.left_trigger);
                if (gamepad1.left_trigger != 0) {
                    launcher = 1;
                } else {
                    launcher = 0;
                }
                //telemetry.addData("auto", auto);
                telemetry.addData("launcher", launcher);
                telemetry.addData("pickup", pickup);
                telemetry.addData("middle", middle);
                telemetry.addData("feeder", feeder);
                telemetry.addData("claw", claw);
                telemetry.addData("drawbridge", drawbridge);
                telemetry.addData("left", LeftY);
                telemetry.addData("right", RightY);
                telemetry.addData("Motor1", robot.leftDrive.getVelocity());
                telemetry.addData("Motor2", robot.rightDrive.getVelocity());
                telemetry.addData("Motor1Pos", robot.leftDrive.getCurrentPosition());
                telemetry.addData("Motor2Pos", robot.rightDrive.getCurrentPosition());
                telemetry.addData("Motor3", robot.leftShooter.getVelocity());
                telemetry.addData("Motor4", robot.rightShooter.getVelocity());
                telemetry.update();
            }
        }
    }
}