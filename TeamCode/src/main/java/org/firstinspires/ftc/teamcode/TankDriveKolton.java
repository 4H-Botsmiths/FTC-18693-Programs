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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name = "TankDriveKolton", group = "Iterative Opmode")
//@Disabled
public class TankDriveKolton extends OpMode {
    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    RobotHardware robot = new RobotHardware();
    // 0= off, 1= on
    public double shooterOn = (0);
    public String detectedColor;

    public void Ramp() {
        if (gamepad2.right_trigger > 0.9){
            robot.leftShooter.setVelocity(robot.shootVelocity);
            robot.rightShooter.setVelocity(robot.shootVelocity);
            robot.rampBottom.setPower(0.75);
            robot.rampMiddle.setPower(0.75);
            robot.rampTop.setPower(0.75);
        }else if (gamepad2.right_trigger >0.5){
            robot.leftShooter.setVelocity(robot.shootVelocity*0.5);
            robot.rightShooter.setVelocity(robot.shootVelocity*0.5);
            robot.rampBottom.setPower(0.75);
            robot.rampMiddle.setPower(0.75);
            robot.rampTop.setPower(0);
        } else if (gamepad2.right_trigger > 0){
            robot.leftShooter.setVelocity(robot.shootVelocity*0.25);
            robot.rightShooter.setVelocity(robot.shootVelocity*0.25);
            robot.rampBottom.setPower(0.75);
            robot.rampMiddle.setPower(0);
            robot.rampTop.setPower(0);
        }else {
            robot.leftShooter.setVelocity(0);
            robot.rightShooter.setVelocity(0);
            robot.rampBottom.setPower(0);
            robot.rampMiddle.setPower(0);
            robot.rampTop.setPower(0);
        }

        }

    short gamepad1=1;
    public void Telemetries() {
        telemetry.addData("Drive Velocity", "Left (%.2f), Right (%.2f)", robot.leftDrive.getVelocity(), robot.rightDrive.getVelocity());
        telemetry.addData("Shooter Velocity", "Left (%.2f), Right (%.2f)", robot.leftDrive.getVelocity(), robot.rightDrive.getVelocity());
        telemetry.addData("Ramp Power", "Bottom (%.2f), Middle (%.2f), Top (%.2f)", robot.rampBottom.getPower(), robot.rampMiddle.getPower(), robot.rampTop.getPower());
        telemetry.addData("Claw Power", "Arm (%.2f), Hand (%.2f)", robot.clawArm.getPower(), robot.clawHand.getPower());
       telemetry.addData("Distance", "left %.2f, right %.2f", robot.distanceLeft.getDistance(DistanceUnit.METER), robot.distanceRight.getDistance(DistanceUnit.METER));
        telemetry.addData("Color Detected", detectColor());
        telemetry.update();
    }
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
            return  "Red";
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
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robot.init(hardwareMap);
        telemetry.addData("Status", "Initializing...");
        telemetry.update();



    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
        telemetry.addData("Status", "Running");
        telemetry.update();

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        //function.detectColor();
        if (robot.voltageSensor.getVoltage() < robot.lowBattery){
            robot.driveVelocity = robot.maxDriveVelocity*0.75;
        } else if (robot.voltageSensor.getVoltage() < robot.reallyLowBattery){
            robot.shootVelocity = robot.maxShootVelocity*0.75;
            robot.driveVelocity = robot.maxDriveVelocity*0.5;
        } else{
            robot.shootVelocity = robot.maxShootVelocity;
            robot.driveVelocity = robot.maxDriveVelocity;
        }
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftVelocity = (-gamepad1.left_stick_y * robot.driveVelocity);
        double rightVelocity = (-gamepad1.right_stick_y * robot.driveVelocity);
        robot.leftDrive.setVelocity(leftVelocity);
        robot.rightDrive.setVelocity(rightVelocity);


       // function.Telemetries();

        if (robot.touchTop.isPressed()) {
            robot.clawArm.setPower(0);
            robot.clawHand.setPower(0);
        } else if (robot.touchBottom.isPressed()) {
            robot.clawArm.setPower(0);
            robot.clawHand.setPower(0);
        } else {
            robot.clawArm.setPower(-gamepad2.left_stick_y);
            robot.clawHand.setPower(robot.clawArm.getPower()/2);
        }
        if(time%2 == 0){
            robot.greenLight.enableLight(false);
        } else{
            robot.greenLight.enableLight(true);
        }

        detectColor();
        Telemetries();
        Ramp();

    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        telemetry.addData("Status", "Stopping...");
        telemetry.update();
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
        telemetry.update();
    }

}
