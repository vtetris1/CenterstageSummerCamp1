/* Copyright (c) 2022 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.common.hardware;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


public class RobotHardware {

    /* Declare OpMode members. */
    HardwareMap hwMap =  null;

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    public DcMotor frontLeftDrive   = null;
    public DcMotor frontRightDrive   = null;
    public DcMotor backRightDrive   = null;
    public DcMotor backLeftDrive   = null;
    public DcMotor linearSlider = null;

    public Servo tiltServo = null;
    public Servo grabServo = null;

    public Servo airplaneLauncher = null;

    BNO055IMU imu;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public RobotHardware() {}

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     *
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void init(HardwareMap ahwMap)    {
        // save reference to hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        frontLeftDrive  = hwMap.get(DcMotor.class, "motorfl");
        frontRightDrive = hwMap.get(DcMotor.class, "motorfr");
        backLeftDrive  = hwMap.get(DcMotor.class, "motorbl");
        backRightDrive = hwMap.get(DcMotor.class, "motorbr");
        linearSlider = hwMap.get(DcMotor.class, "motorls");


        // set Brake zero power behavior
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // reverse motor directions
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);




        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        setDrivetrainMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        tiltServo = hwMap.get(Servo.class, "tiltServo");
        grabServo = hwMap.get(Servo.class, "grabServo");
        airplaneLauncher = hwMap.get(Servo.class, "launcher");

        // imu parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO0155IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hwMap.get(BNO055IMU.class,"imu");
        imu.initialize(parameters);


    }
    public void setDrivetrainMode(DcMotor.RunMode mode) {
        frontLeftDrive.setMode(mode);
        frontRightDrive.setMode(mode);
        backLeftDrive.setMode(mode);
        backRightDrive.setMode(mode);
    }
    public void setArmsMode(DcMotor.RunMode mode) {
        linearSlider.setMode(mode);
    }


    public void setDrivePower(double fl, double fr, double bl, double br) {
        if (fl > 1.0)
            fl = 1.0;
        else if (fl < -1.0)
            fl = -1.0;

        if (fr > 1.0)
            fr = 1.0;
        else if (fr < -1.0)
            fr = -1.0;

        if (bl > 1.0)
            bl = 1.0;
        else if (bl < -1.0)
            bl = -1.0;

        if (br > 1.0)
            br = 1.0;
        else if (br < -1.0)
            br = -1.0;

        frontLeftDrive.setPower(fl);
        frontRightDrive.setPower(fr);
        backLeftDrive.setPower(bl);
        backRightDrive.setPower(br * (-1)); //had to manually reverse (the -1 reversed it) (Line:93)

    }
    public void setAllDrivePower(double p){ setDrivePower(p,p,p,p);}
    public void setArmPower(double armPower){
        linearSlider.setPower(armPower);
    }






}
/* port 1 fl
   port 2 bl
   port 0 arm
   extension hub port 2 fr
   extension hub port 3 br
   launch 0 exttension hub
   tilt 1 extention hub
   grab 2 extention hub
 */