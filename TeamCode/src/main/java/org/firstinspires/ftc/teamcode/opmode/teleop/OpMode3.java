package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

//test
@TeleOp(name = "OpMode3")

public class OpMode3 extends LinearOpMode {
    public ElapsedTime mRunTime = new ElapsedTime();

    RobotHardware robot = new RobotHardware();

    private int sleepMs1 = 0;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        // Put initialization blocks here.
        waitForStart();
        while (opModeIsActive()) {
            double horizontal = gamepad1.left_stick_x * 0.8;
            double vertical = -gamepad1.left_stick_y * 0.8;
            double turn = gamepad1.right_stick_x * 0.8;

            robot.setDrivePower(vertical + turn - horizontal, vertical - turn + horizontal, vertical + turn + horizontal, vertical - turn - horizontal);

            telemetry.addLine(String.format("FL: %d \nBL %d \nFR: %d \nBR: %d ",
                    robot.motorfl.getCurrentPosition(),
                    robot.motorbl.getCurrentPosition(),
                    robot.motorfr.getCurrentPosition(),
                    robot.motorbr.getCurrentPosition()
            ));

            //lift arm start
            if (gamepad1.a) { //if button a pressed
                //tilt the lift to be upright
                robot.liftArm.setPower(-0.5);   //set motor power
                sleep(1000);             // let motor run for some time seconds.
                robot.liftArm.setPower(0);   //set lower motor power to maintain the position
            }

            if (gamepad2.y) {
                robot.liftHex.setPower(0.5);   //set motor power
                sleep(1000);             // let motor run for some time seconds.
                robot.liftHex.setPower(0);   //set lower motor power to maintain the position

            }

            if (gamepad1.x) { //if button a pressed
                // Extend liftArm
                robot.liftHex.setPower(0.5);
                sleep(250);             // let motor run for some time seconds.
                robot.liftArm.setPower(0);
            }
            if (gamepad1.b) { //if button a pressed
                // Retract liftArm
                robot.liftHex.setPower(-0.4);
                sleep(300);             // let motor run for some time seconds.
                robot.liftArm.setPower(0);
            }


            // Launch airplane
            /*
            if (gamepad1.right_trigger > 0.5) {
                // Tilt the launcher in a given degree in order to launch airplane over the bar
                TiltLiftOne(-0.5, (int) (300 * 2.5), -0.1, 0.8, (int) (350 * 1.6), 0);

                robot.launcher.setPower(-1.0);
                sleep(3000);
                robot.airplaneFeeder.setPosition(0);
                sleep(1000);
                robot.launcher.setPower(0.0);
                robot.airplaneFeeder.setPosition(0.5);
            }
            */


//grabber
            if (gamepad2.left_trigger > 0.5) {
                robot.grabServoLeft.setPosition(1.0); // open

            } else if (gamepad2.left_bumper) {
                robot.grabServoLeft.setPosition(0.0); // close


            }

            if (gamepad2.right_trigger > 0.5) {

                robot.grabServoRight.setPosition(0.0); // open
            } else if (gamepad2.right_bumper) {

                robot.grabServoRight.setPosition(1.0); // close
            }


//tilt arm
            if (gamepad2.left_stick_y > 0.7) {
                robot.liftHex.setPower(-0.3);
                robot.setDrivePower(vertical + turn - horizontal, vertical - turn + horizontal, vertical + turn + horizontal, vertical - turn - horizontal);

            }

            else if (gamepad2.left_stick_y < -0.7) {
                robot.liftHex.setPower(0.5);
                robot.setDrivePower(vertical + turn - horizontal, vertical - turn + horizontal, vertical + turn + horizontal, vertical - turn - horizontal);

            }

            else {
                robot.liftHex.setPower(0);
            }

//tilt servo
            if (gamepad2.right_stick_y > 0.7) {
                robot.tiltServoLeft.setPosition(1.0);

            } else if (gamepad2.right_stick_y < -0.7) {
                robot.tiltServoLeft.setPosition(0);
            }


        }


        //emergency releases
    }

        private void TiltLiftOne ( double crankPowerBegin, int crankTimeMs, double crankPowerEnd,
        double liftPowerBegin, int liftTimeMs, double liftPowerEnd){
            //tilt the lift to be upright
            robot.liftHex.setPower(crankPowerBegin);   //set motor power
            sleep(crankTimeMs);          // let motor run for some time seconds.
            robot.liftHex.setPower(crankPowerEnd);   //set lower motor power to maintain the position

            // Extend liftArm
            robot.liftArm.setPower(liftPowerBegin);
            sleep(liftTimeMs);             // let motor run for some time seconds.
            robot.liftArm.setPower(liftPowerEnd);
        }

    }


