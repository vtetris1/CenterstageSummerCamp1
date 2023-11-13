package org.firstinspires.ftc.teamcode.opmode.teleop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;


@TeleOp(name = "OpMode2")

public class OpMode extends LinearOpMode {

    RobotHardware robot = new RobotHardware();

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        // Put initialization blocks here.
        waitForStart();
        while (opModeIsActive()) {
            double horizontal = gamepad1.left_stick_x * 1.0;
            double vertical = -gamepad1.left_stick_y * 1.0;
            double turn = gamepad1.right_stick_x * 1.0;

            robot.setDrivePower(vertical+turn-horizontal,vertical-turn+horizontal,vertical+turn+horizontal,vertical-turn-horizontal);

            if (-gamepad2.left_stick_y > 0.1){
                robot.setArmPower(0.45);
            }
            else if (-gamepad2.left_stick_y < -0.1){
                robot.setArmPower(-0.45);
            }
            else{
                robot.setArmPower(0);
            }

            //servos

            if (gamepad2.left_trigger > 0.5) {
                robot.grabServo.setPosition(0.4); // open
            }
            else if (gamepad2.right_trigger>0.5){
                robot.grabServo.setPosition(1); // close
            }


            if (gamepad1.a) {
                robot.airplaneLauncher.setPosition(0.5); //works
            }

            //tilt servo
            double initTilt = 0.0;

            if (gamepad2.right_stick_y > 0.7) {
                initTilt += 0.1;

                if (initTilt > 1.0) {
                    initTilt = 1.0;
                    robot.tiltServo.setPosition(-initTilt);
                }

                else robot.tiltServo.setPosition(-initTilt);
            }

            if (gamepad2.right_stick_y < -0.7) {
                initTilt -= 0.1;

                if (initTilt < -1.0) {
                    initTilt = -1.0;
                    robot.tiltServo.setPosition(initTilt);
                }

                else robot.tiltServo.setPosition(initTilt);
            }

            telemetry.update();
        }
    }
}