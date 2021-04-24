package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

public class RobotFunctions {
    float Red;
    float Green;
    float Blue;
    double counter;
    double RedFactor;
    double GreenFactor;
    double Blue_Factor;
    int mm_tick;
    int mm_in;
    RobotHardware robot = new RobotHardware();
    public void Telemetries() {
        telemetry.addData(String.valueOf(robot.leftDrive), "Did this work?");
    }
    private void Move(int in, boolean forward_, boolean moveoverride) {
        while (FixMeWilliam())
            telemetry.addData("Move Waiting", counter);
            Telemetries();
            counter += 1;
            if (forward_) {
                robot.leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
                robot.rightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
            } else {
                robot.leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
                robot.rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
            }
            if (moveoverride || !robot.rightDrive.isBusy()) {
                counter = 0;
                robot.leftDrive.setPower(0);
                robot.rightDrive.setPower(0);
                robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.leftDrive.setTargetPosition(in * mm_in * mm_tick);
                robot.rightDrive.setTargetPosition(in * mm_in * mm_tick);
                robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                // set F to 54.1
                robot.leftDrive.setVelocityPIDFCoefficients(0.0075, 0.000075, 0.005, 54.1);
                robot.rightDrive.setVelocityPIDFCoefficients(0.0075, 0.000075, 0.005, 54.1);
                robot.leftDrive.setPower(1);
                robot.rightDrive.setPower(1);
            }
        }
    }

