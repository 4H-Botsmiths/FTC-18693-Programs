/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.android.AndroidSoundPool;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.TempUnit;

import static android.os.SystemClock.sleep;

@TeleOp(name = "Teleop Kolton", group = "Kolton")
//Selection Code. Runs Once
public class TeleopKolton extends OpMode {
    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    public AndroidSoundPool audio;
    public String detectedColor;
    public double Status = 5;
    public boolean mutantGamepad = false;
    //Declare Robot Variable. (See RobotHardware.java)
    RobotHardware robot = new RobotHardware();

    //Declare Functions ⬇️

    //Raise Ramp Based Off Of Gamepad Input
    public void RampUp(float Trigger, boolean Bumper) {
        if (Bumper) {
            robot.rampBottom.setPower(0.75);
            robot.rampMiddle.setPower(0);
            robot.rampTop.setPower(0);
        } else if (Trigger > 0.9) {
            robot.leftShooter.setVelocity(robot.shootVelocity);
            robot.rightShooter.setVelocity(robot.shootVelocity);
            robot.rampBottom.setPower(0.75);
            robot.rampMiddle.setPower(0.75);
            robot.rampTop.setPower(0.75);
        } else if (Trigger > 0.5) {
            //    robot.leftShooter.setVelocity(robot.shootVelocity * 0.5);
            //    robot.rightShooter.setVelocity(robot.shootVelocity * 0.5);
            robot.rampBottom.setPower(0.75);
            robot.rampMiddle.setPower(0.75);
            robot.rampTop.setPower(0);
        } else if (Trigger > 0) {
            //    robot.leftShooter.setVelocity(robot.shootVelocity * 0.25);
            //    robot.rightShooter.setVelocity(robot.shootVelocity * 0.25);
            robot.rampBottom.setPower(0.75);
            robot.rampMiddle.setPower(0);
            robot.rampTop.setPower(0);
        } else {
            robot.leftShooter.setVelocity(0);
            robot.rightShooter.setVelocity(0);
            robot.rampBottom.setPower(0);
            robot.rampMiddle.setPower(0);
            robot.rampTop.setPower(0);
        }

    }

    //Lower Ramp Based Off Of Gamepad Input
    public void RampDown(float Trigger) {
        if (Trigger > 0.9) {
            robot.rampTop.setPower(-0.75);
            robot.rampMiddle.setPower(-0.75);
            robot.rampBottom.setPower(-0.75);
        } else if (Trigger > 0.5) {
            robot.rampTop.setPower(-0.75);
            robot.rampMiddle.setPower(-0.75);
            robot.rampBottom.setPower(0);
        } else if (Trigger > 0) {
            robot.rampTop.setPower(-0.75);
            robot.rampMiddle.setPower(0);
            robot.rampBottom.setPower(0);
        } else {
            robot.leftShooter.setVelocity(0);
            robot.rightShooter.setVelocity(0);
            robot.rampBottom.setPower(0);
            robot.rampMiddle.setPower(0);
            robot.rampTop.setPower(0);
        }

    }

    //Update Android Screen
    public void Telemetries() {
        if (Status == 2) {
            telemetry.addData("Status", "Danger! Really Low Voltage");
        } else if (Status == 1) {
            telemetry.addData("Status", "WARNING! Low Voltage");
        } else if (mutantGamepad) {
            telemetry.addData("Status", "Running, Mutant Gamepad Enabled");
        } else {
            telemetry.addData("Status", "Running");
        }
        telemetry.addData("Drive Velocity", "Left (%.2f%%), Right (%.2f%%)", robot.leftDrive.getVelocity() / robot.driveVelocity * 100, robot.rightDrive.getVelocity() / robot.driveVelocity * 100);
        telemetry.addData("Shooter Velocity", "Left (%.2f%%), Right (%.2f%%)", robot.leftShooter.getVelocity() / robot.shootVelocity * 100, robot.rightShooter.getVelocity() / robot.shootVelocity * 100);
        telemetry.addData("Ramp Power", "Bottom (%.2f%%), Middle (%.2f%%), Top (%.2f%%)", robot.rampBottom.getPower() / robot.servoPower * 100, robot.rampMiddle.getPower() / robot.servoPower * 100, robot.rampTop.getPower() / robot.servoPower * 100);
        telemetry.addData("Claw Power", "Arm (%.2f), Hand (%.2f)", robot.clawArm.getPower() * 100, robot.clawHand.getPower() * 100);
        telemetry.addData("Distance", "left %.2f, right %.2f", robot.distanceLeft.getDistance(DistanceUnit.METER), robot.distanceRight.getDistance(DistanceUnit.METER));
        telemetry.addData("Color Detected", detectColor());

        //telemetry.addData("Temperature","%.2f", robot.gyro.getTemperature().toUnit(TempUnit.FARENHEIT));
    }

    //Detect And Return Color Underneath The Robot In Red, Green, Yellow etc. format
    public String detectColor() {
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
            return "Red";
        } else if (hue < 60) {
            detectedColor = "Orange";
            return "Orange";
        } else if (hue < 90) {
            detectedColor = "Yellow";
            return "Yellow";
        } else if (hue < 150) {
            detectedColor = "Green";
            return "Green";
        } else if (hue < 225) {
            detectedColor = "Blue";
            return "Blue";
        } else if (hue < 350) {
            detectedColor = "Purple";
            return "Purple";
        } else {
            detectedColor = "Not Detected";
            return "Not Detected";
        }
    }


    @Override
    //Initialization Code. Runs Once Per Execution
    public void init() {
        telemetry.addData("Status", "Initializing...");
        telemetry.addData("Gyro", "calibrating...");
        robot.init(hardwareMap);
        audio = new AndroidSoundPool();
        audio.initialize(SoundPlayer.getInstance());


    }


    @Override
    //Initialization Code. Runs Repeatedly Until Start
    public void init_loop() {
        telemetry.addData("Gyro", robot.gyro.getCalibrationStatus().toString());
        telemetry.addData("Status", "Initialized");
    }


    @Override
    //Start Code. Runs Once
    public void start() {
        runtime.reset();
        telemetry.addData("Status", "Running");
        telemetry.update();

    }


    @Override
    //Start Code. Runs Repeatedly Until Stop
    public void loop() {
        //function.detectColor();
        // Setup a variable for each drive wheel to save power level for telemetry
        /* double leftVelocity = (Math.pow(-gamepad1.left_stick_y, 7) * robot.driveVelocity);
        double rightVelocity = (Math.pow(-gamepad1.right_stick_y, 7) * robot.driveVelocity);
        if (leftVelocity/robot.driveVelocity < 0.1 && leftVelocity/robot.driveVelocity > 0.01) {leftVelocity = 0.1 * robot.driveVelocity;}
        if (rightVelocity/robot.driveVelocity < 0.1 && rightVelocity/robot.driveVelocity > 0.01) {rightVelocity = 0.1 * robot.driveVelocity;}
        */
        robot.leftDrive.setVelocity(-gamepad1.left_stick_y * robot.driveVelocity);
        robot.rightDrive.setVelocity(-gamepad1.right_stick_y * robot.driveVelocity);

        if (robot.voltageSensor.getVoltage() < robot.reallyLowBattery && Status != 2) {
            if (Status != 1) audio.play("RawRes:ss_siren");
            Status = 2;
            robot.shootVelocity = robot.maxShootVelocity * 0.75;
            robot.driveVelocity = robot.maxDriveVelocity * 0.5;
        } else if (robot.voltageSensor.getVoltage() < robot.lowBattery && Status != 1) {
            if (Status != 2) audio.play("RawRes:ss_siren");
            Status = 1;
            robot.driveVelocity = robot.maxDriveVelocity * 0.75;
        } else {
            Status = 0;
            robot.shootVelocity = robot.maxShootVelocity;
            robot.driveVelocity = robot.maxDriveVelocity;
            audio.stop();
        }

        // function.Telemetries();

        if (robot.touchTop.isPressed()) {
            robot.clawArm.setPower(1);
            robot.clawHand.setPower(robot.clawArm.getPower() / robot.armRatio);
        } else if (robot.touchBottom.isPressed()) {
            robot.clawArm.setPower(-1);
            robot.clawHand.setPower(robot.clawArm.getPower() / robot.armRatio);
        } else if (gamepad2.a) {
            robot.clawHand.setPower(-1);
        } else if (gamepad2.y) {
            robot.clawHand.setPower(1);
        } else if (gamepad2.dpad_up) {
            robot.clawArm.setPower(1);
            robot.clawHand.setPower(-robot.clawArm.getPower() / robot.armRatio);
        } else if (gamepad2.dpad_down) {
            robot.clawArm.setPower(-1);
            robot.clawHand.setPower(-robot.clawArm.getPower() / robot.armRatio);
        } else if (gamepad2.left_stick_y != 0 || gamepad2.right_stick_y != 0) {
            robot.clawArm.setPower(-gamepad2.left_stick_y);
            robot.clawHand.setPower(-gamepad2.right_stick_y);
        } else if (mutantGamepad) {
            robot.clawArm.setPower(-gamepad1.left_stick_x);
            robot.clawHand.setPower(-gamepad1.right_stick_x);
        } else {
            robot.clawHand.setPower(0);
            robot.clawArm.setPower(0);
        }


        robot.greenLight.enableLight(time % 2 != 0);

        if (gamepad2.right_bumper) {
            RampUp(0, true);
        } else if (gamepad2.right_trigger > 0) {
            RampUp(gamepad2.right_trigger, false);
        } else if (gamepad1.right_bumper && mutantGamepad) {
            RampUp(0, true);
        } else if (gamepad1.right_trigger > 0 && mutantGamepad) {
            RampUp(gamepad1.right_trigger, false);
        } else if (gamepad2.left_trigger > 0) {
            robot.leftShooter.setPower(0);
            robot.rightShooter.setPower(0);
            RampDown(gamepad2.left_trigger);
        } else if (gamepad1.left_trigger > 0 && mutantGamepad) {
            robot.leftShooter.setPower(0);
            robot.rightShooter.setPower(0);
            RampDown(gamepad1.left_trigger);
        } else {
            robot.rampTop.setPower(0);
            robot.rampMiddle.setPower(0);
            robot.rampBottom.setPower(0);
            robot.leftShooter.setPower(0);
            robot.rightShooter.setPower(0);
        }
        if (gamepad1.back || gamepad2.back) {
            if (mutantGamepad) {
                mutantGamepad = false;
            } else {
                mutantGamepad = true;
            }
        }

        Telemetries();
    }



    /*    if(gamepad1.a){
            if (Direction){
                robot.maxDriveVelocity = -288;
                Direction = false;
            }else {
                robot.maxDriveVelocity = 288;
                Direction = true;
            }
        }
*/


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    //Stop Code. Runs Once
    public void stop() {
        telemetry.addData("Status", "Stopping...");
        audio.close();
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        robot.leftShooter.setPower(0);
        robot.leftShooter.setPower(0);
        robot.rampBottom.setPower(0);
        robot.rampBottom.setPower(0);
        robot.rampBottom.setPower(0);
        robot.clawArm.setPower(0);
        robot.clawHand.setPower(0);
        telemetry.addData("Status", "Stopped");
        robot.greenLight.enableLight(false);
        telemetry.update();
    }

}
