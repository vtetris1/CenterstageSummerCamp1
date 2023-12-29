package org.firstinspires.ftc.teamcode.autonomous;
//test

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

//ignore this for now
@Autonomous(name="Red_Far_v1_F2_E6")
public class Red_Far_v1_F2_E6 extends LinearOpMode {
    RobotHardware robot = new RobotHardware();
    // Motor encoder parameter
    double ticksPerInch = 31.3;
    double ticksPerDegree = 15.6;


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        //reset encoder
        robot.setAutoDriveMotorMode();

        telemetry.addLine(String.format("DistanceR: %.1f inch\nDistanceL: %.1f inch\n",
                robot.distanceR.getDistance(DistanceUnit.INCH),
                robot.distanceL.getDistance(DistanceUnit.INCH)));
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            telemetry.update();

            int forwardTicks = 1215;
            driveMotors(forwardTicks, forwardTicks, forwardTicks, forwardTicks, 0.35,
                    true, robot.yaw0);
            sleep(1000);

            if (robot.distanceL.getDistance(DistanceUnit.INCH) < 10) {

                forwardTicks = 100;
                driveMotors(forwardTicks, forwardTicks, forwardTicks, forwardTicks, 0.3,
                        true, robot.yaw0);

                sleep(100);

                turnToTargetYaw(90 + robot.yaw0, 0.4, 6500);

                sleep(100);

                forwardTicks = -200;
                driveMotors(forwardTicks, forwardTicks, forwardTicks, forwardTicks, 0.3,
                        true, robot.yaw0);

                sleep(500);

                robot.autoPixel.setPosition(0.0);

                sleep(350);

                turnToTargetYaw(0 + robot.yaw0, 0.4, 8000);

                sleep(100);

                forwardTicks = 1000;
                driveMotors(forwardTicks, forwardTicks, forwardTicks, forwardTicks, 0.3,
                        true, robot.yaw0);

                int sideTicks = 600;
                driveMotors((int)(sideTicks*1.2), -sideTicks, -sideTicks, (int)(sideTicks*1.2), 0.3,
                        true, robot.yaw0);

                sleep(100);

                forwardTicks = 5000;
                driveMotors(forwardTicks, forwardTicks, forwardTicks, forwardTicks, 0.4,
                        true, robot.yaw0);

                //sleep(100);

                robot.boardPixel.setPosition(1.0);

                sleep(100);

                robot.boardPixel.setPosition(0.0);

                //requestOpModeStop();
            }
            else if (robot.distanceR.getDistance(DistanceUnit.INCH) < 10) {
                forwardTicks = 85;
                driveMotors(forwardTicks, forwardTicks, forwardTicks, forwardTicks, 0.35,
                        true, robot.yaw0);

                sleep(100);

                turnToTargetYaw(-60 + robot.yaw0, 0.42, 6000);

                sleep(100);

                forwardTicks = 250;
                driveMotors(forwardTicks, forwardTicks, forwardTicks, forwardTicks, 0.3,
                        true, robot.yaw0);

                sleep(100);

                robot.autoPixel.setPosition(0.0);

                sleep(1000);

                forwardTicks = -150;
                driveMotors(forwardTicks, forwardTicks, forwardTicks, forwardTicks, 0.3,
                        true, robot.yaw0);



                //forwardTicks = -200;
                //driveMotors(forwardTicks, forwardTicks, forwardTicks, forwardTicks, 0.3,
                //         true, robot.yaw0);

                //sleep(1000);

                turnToTargetYaw(0 + robot.yaw0, 0.4, 2000);

                sleep(100);

                forwardTicks = 1050;
                driveMotors(forwardTicks, forwardTicks, forwardTicks, forwardTicks, 0.45,
                        true, robot.yaw0);

                turnToTargetYaw(-90 + robot.yaw0, 0.42, 5000);
                //int sideTicks = 1000;
                // 1&4: value, 2&3: -value
                //driveMotors((int)(sideTicks*1.2), -sideTicks, -sideTicks, (int)(sideTicks*1.2), 0.25,
                //        true, robot.yaw0);

                //sleep(100);

                forwardTicks = 4000;
                driveMotors(forwardTicks, forwardTicks, forwardTicks, forwardTicks, 0.55,
                        true, robot.yaw0);

                sleep(100);

                robot.boardPixel.setPosition(1.0);

                robot.boardPixel.setPosition(0.0);
                //requestOpModeStop();



            } else {
                forwardTicks = 225;
                driveMotors(forwardTicks, forwardTicks, forwardTicks, forwardTicks, 0.3,
                        true, robot.yaw0);

                sleep(100);

                robot.autoPixel.setPosition(0.0);

                sleep(700);

                forwardTicks = -150;
                driveMotors(forwardTicks, forwardTicks, forwardTicks, forwardTicks, 0.3,
                        true, robot.yaw0);

                sleep(500);

                turnToTargetYaw(-90 + robot.yaw0, 0.4, 6000);

                sleep(500);

                forwardTicks = 3000;
                driveMotors(forwardTicks, forwardTicks, forwardTicks, forwardTicks, 0.3,
                        true, robot.yaw0);

                sleep(100);

                robot.boardPixel.setPosition(0.0);

                robot.boardPixel.setPosition(1.0);
            }


            while (opModeIsActive()) {
                telemetry.addLine(String.format("DistanceR: %.1f inch\nDistanceL: %.1f inch\nCurrent Yaw: %.1f",
                        robot.distanceR.getDistance(DistanceUnit.INCH),
                        robot.distanceL.getDistance(DistanceUnit.INCH),
                        robot.getCurrentYaw()));
                telemetry.update();
            }
        }
    }

    private void driveMotors(int flTarget, int blTarget, int frTarget, int brTarget,
                             double power,
                             boolean bKeepYaw, double targetYaw){
        double currentYaw, diffYaw;
        double powerDeltaPct, powerL, powerR;
        double leftRatioToCounterCOG = 1.0;
        int direction;

        robot.motorfl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorbl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorfr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorbr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.motorfl.setTargetPosition(flTarget);
        robot.motorbl.setTargetPosition(blTarget);
        robot.motorfr.setTargetPosition(frTarget);
        robot.motorbr.setTargetPosition(brTarget);

        robot.motorfl.setPower(power * leftRatioToCounterCOG);
        robot.motorbl.setPower(power * leftRatioToCounterCOG);
        robot.motorfr.setPower(power);
        robot.motorbr.setPower(power);

        robot.motorfl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorbl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorfr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorbr.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Defensive programming.
        // Use bKeepYaw only when all targets are the same, meaning moving in a straight line
        if (! ((flTarget == blTarget)
                && (flTarget == frTarget)
                && (flTarget == brTarget)) )
            bKeepYaw = false;
        direction = (flTarget > 0) ? 1 : -1;
        while(opModeIsActive() &&
                (robot.motorfl.isBusy() &&
                        robot.motorbl.isBusy() &&
                        robot.motorfr.isBusy() &&
                        robot.motorbr.isBusy())){
            if (bKeepYaw) {

                currentYaw = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                if (Math.abs(currentYaw - targetYaw) > 2.0)
                    powerDeltaPct = 0.25;
                else
                    powerDeltaPct = Math.abs(currentYaw - targetYaw) / 2.0 * 0.25;
                if (currentYaw < targetYaw) {
                    powerL = power * (1 - direction * powerDeltaPct);
                    powerR = power * (1 + direction * powerDeltaPct);
                }
                else {
                    powerL = power * (1 + direction * powerDeltaPct);
                    powerR = power * (1 - direction * powerDeltaPct);
                }
                if (powerL > 1.0)
                    powerL = 1.0;
                if (powerR > 1.0)
                    powerR = 1.0;
                robot.motorfl.setPower(powerL);
                robot.motorbl.setPower(powerL);
                robot.motorfr.setPower(powerR);
                robot.motorbr.setPower(powerR);
            }
            idle();
        }

        robot.motorfl.setPower(0);
        robot.motorbl.setPower(0);
        robot.motorfr.setPower(0);
        robot.motorbr.setPower(0);
    }

    private void nonEncoderDrive(){

    }

    private void driveStrafe(int flTarget, int blTarget, int frTarget, int brTarget,
                             double power,
                             boolean bKeepYaw, double targetYaw){
        double currentYaw, diffYaw;
        double powerDeltaPct, powerL, powerR;
        int direction;

        robot.motorfl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorbl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorfr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorbr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.motorfl.setTargetPosition(flTarget);
        robot.motorbl.setTargetPosition(blTarget);
        robot.motorfr.setTargetPosition(frTarget);
        robot.motorbr.setTargetPosition(brTarget);

        robot.motorfl.setPower(power);
        robot.motorbl.setPower(power);
        robot.motorfr.setPower(power);
        robot.motorbr.setPower(power);

        robot.motorfl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorbl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorfr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorbr.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Defensive programming.
        // Use bKeepYaw only when all targets are the same, meaning moving in a straight line
        if (! ((flTarget == blTarget)
                && (flTarget == frTarget)
                && (flTarget == brTarget)) )
            bKeepYaw = false;
        direction = (flTarget > 0) ? 1 : -1;
        while(opModeIsActive() &&
                (robot.motorfl.isBusy() &&
                        robot.motorbl.isBusy() &&
                        robot.motorfr.isBusy() &&
                        robot.motorbr.isBusy())){
            if (bKeepYaw) {

                currentYaw = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                if (Math.abs(currentYaw - targetYaw) > 2.0)
                    powerDeltaPct = 0.25;
                else
                    powerDeltaPct = Math.abs(currentYaw - targetYaw) / 2.0 * 0.25;
                if (currentYaw < targetYaw) {
                    powerL = power * (1 - direction * powerDeltaPct);
                    powerR = power * (1 + direction * powerDeltaPct);
                }
                else {
                    powerL = power * (1 + direction * powerDeltaPct);
                    powerR = power * (1 - direction * powerDeltaPct);
                }
                if (powerL > 1.0)
                    powerL = 1.0;
                if (powerR > 1.0)
                    powerR = 1.0;
                robot.motorfl.setPower(powerL);
                robot.motorbl.setPower(powerL);
                robot.motorfr.setPower(powerR);
                robot.motorbr.setPower(powerR);
            }
            idle();
        }

        robot.motorfl.setPower(0);
        robot.motorbl.setPower(0);
        robot.motorfr.setPower(0);
        robot.motorbr.setPower(0);
    }


    private void turnToTargetYaw(double targetYawDegree, double power, long maxAllowedTimeInMills){
        long timeBegin, timeCurrent;
        double currentYaw = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);;
        int ticks, tickDirection;
        double factor = 1.0;

        double diffYaw = Math.abs(currentYaw - targetYawDegree);
        telemetry.addLine(String.format("\nCurrentYaw=%.2f\nTargetYaw=%.2f", currentYaw, targetYawDegree));
        telemetry.update();

        timeBegin = timeCurrent = System.currentTimeMillis();
        while (diffYaw > 0.5
                && opModeIsActive()
                && ((timeCurrent-timeBegin) < maxAllowedTimeInMills)) {
            ticks = (int) (diffYaw * ticksPerDegree);
            if (ticks > 150)
                ticks = 150;

            tickDirection = (currentYaw < targetYawDegree) ? -1 : 1;
            if (ticks < 1)
                break;
            if (diffYaw > 3)
                factor = 1.0;
            else
                factor = diffYaw / 3;
            driveMotors(
                    (int)(tickDirection * ticks),
                    (int)(tickDirection * ticks),
                    -(int)(tickDirection * ticks),
                    -(int)(tickDirection * ticks),
                    power * factor, false, 0);
            currentYaw = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            timeCurrent = System.currentTimeMillis();
            diffYaw = Math.abs(currentYaw - targetYawDegree);

            telemetry.addLine(String.format("\nCurrentYaw=%.2f\nTargetYaw=%.2f\nTimeLapsed=%.2f ms",
                    currentYaw, targetYawDegree, (double)(timeCurrent-timeBegin)));
            telemetry.update();
        }
    }

    private void deployPreloadedPixel1(int timeIntervalMs) {
        // Deploy preloaded pixel 1
        robot.autoPixel.setPosition(1.0);
        sleep(timeIntervalMs);
        robot.autoPixel.setPosition(0.5);
        sleep(timeIntervalMs);

    }

}
