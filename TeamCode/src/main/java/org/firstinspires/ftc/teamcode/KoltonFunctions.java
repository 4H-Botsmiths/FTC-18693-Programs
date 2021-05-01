package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

public class KoltonFunctions {
    RobotHardware robot = new RobotHardware();
    private final ElapsedTime runtime = new ElapsedTime();
    public String detectedColor;


    public void Telemetries() {
        telemetry.addData("Drive Velocity", "Left (%.2f), Right (%.2f)", robot.leftDrive.getVelocity(), robot.rightDrive.getVelocity());
        telemetry.addData("Shooter Velocity", "Left (%.2f), Right (%.2f)", robot.leftDrive.getVelocity(), robot.rightDrive.getVelocity());
        telemetry.addData("Ramp Power", "Bottom (%.2f), Middle (%.2f), Top (%.2f)", robot.rampBottom.getPower(), robot.rampMiddle.getPower(), robot.rampTop.getPower());
        telemetry.addData("Claw Power", "Arm (%.2f), Hand (%.2f)", robot.clawArm.getPower(), robot.clawHand.getPower());
        telemetry.addData("Color Detected", detectedColor);
        telemetry.update();
    }

    public void Ramp() {
       // double shooterSpeed = (robot.leftShooter.getVelocity()+robot.rightShooter.getVelocity())/2;
        double shooterSpeed = 0;
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
    public void detectColor() {
        int colorHSV;
        float hue;
        float sat;
        float val;
        // Convert RGB values to HSV color model.
        // See https://en.wikipedia.org/wiki/HSL_and_HSV for details on HSV color model.
        colorHSV = Color.argb(robot.color1.alpha(), robot.color1.red(), robot.color1.green(), robot.color1.blue());
        // Get hue.
        hue = JavaUtil.colorToHue(colorHSV);
        //telemetry.addData("Hue", hue);
        // Get saturation.
        sat = JavaUtil.colorToSaturation(colorHSV);
        //telemetry.addData("Saturation", sat);
        // Get value.
        val = JavaUtil.colorToValue(colorHSV);
        //telemetry.addData("Value", val);
        // Use hue to determine if it's red, green, blue, etc..
        if (hue < 30) {
            detectedColor = "Red";
        } else if (hue < 60) {
            detectedColor = "Orange";
        } else if (hue < 90) {
            detectedColor = "Yellow";
        } else if (hue < 150) {
            detectedColor = "Green";
        } else if (hue < 225) {
            detectedColor = "Blue";
        } else if (hue < 350) {
            detectedColor = "Purple";
        } else {
            detectedColor = "Not Detected";
        }
        // Check to see if it might be black or white.
        if (sat < 0.2) {
            //telemetry.addData("Check Sat", "Is surface white?");
        }
        if (val < 0.16) {
            //telemetry.addData("Check Val", "Is surface black?");
        }
        //telemetry.update();
    }
}