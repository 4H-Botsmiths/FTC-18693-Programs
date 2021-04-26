package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

public class KoltonFunctions {
    RobotHardware robot = new RobotHardware();
    private final ElapsedTime runtime = new ElapsedTime();

    public void Telemetries() {
        telemetry.addData("Drive Velocity", "Left (%.2f), Right (%.2f)", robot.leftDrive.getVelocity(), robot.rightDrive.getVelocity());
        telemetry.addData("Shooter Velocity", "Left (%.2f), Right (%.2f)", robot.leftDrive.getVelocity(), robot.rightDrive.getVelocity());
        telemetry.addData("Ramp Power", "Bottom (%.2f), Middle (%.2f), Top (%.2f)", robot.rampBottom.getPower(), robot.rampMiddle.getPower(), robot.rampTop.getPower());
        telemetry.addData("Claw Power", "Arm (%.2f), Hand (%.2f)", robot.clawArm.getPower(), robot.clawHand.getPower());
        telemetry.update();
    }

    public void Ramp() {
        double shooterSpeed = (robot.leftShooter.getVelocity()+robot.rightShooter.getVelocity())/2;
        if (shooterSpeed < robot.shootVelocity){
            telemetry.addData("Status", "WARNING!, insufficient Shooter Speed");
        } else {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
        }
        if (-gamepad2.right_stick_y > 0) {
            if (-gamepad2.right_stick_y > 90) {
                robot.rampBottom.setPower(0.5);
                robot.rampMiddle.setPower(0.5);
                robot.rampTop.setPower(0.5);
            } else if (-gamepad2.right_stick_y > 50) {
                robot.rampBottom.setPower(0.5);
                robot.rampMiddle.setPower(0.5);
            } else if (-gamepad2.right_stick_y > 0) {
                robot.rampBottom.setPower(0.5);
            }

        } else if (-gamepad2.right_stick_y < 0) {
            if (-gamepad2.right_stick_y < 0.9) {
                robot.rampBottom.setPower(0.5);
                robot.rampMiddle.setPower(0.5);
                robot.rampTop.setPower(0.5);
            } else if (-gamepad2.right_stick_y < 0.5) {
                robot.rampBottom.setPower(0.5);
                robot.rampMiddle.setPower(0.5);
            } else if (-gamepad2.right_stick_y < 0) {
                robot.rampBottom.setPower(0.5);
            }
        }
    }
}