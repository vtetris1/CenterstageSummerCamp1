package org.firstinspires.ftc.teamcode.autonomous;
//test
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

//ignore this for now
@Autonomous(name="Red_Near_v1_F4_D6")
@Disabled
public class Red_Near_v1_F4_D6 extends LinearOpMode {
    RobotHardware robot = new RobotHardware();
    // Motor encoder parameter
    double ticksPerInch = 6.56;
    double ticksPerDegree = 5.0;


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        //reset encoder
        robot.setAutoDriveMotorMode();

        telemetry.addLine(String.format("DistanceR: %.1f inch\nDistanceL: %.1f inch\nyaw0: %.1f",
                robot.distanceR.getDistance(DistanceUnit.INCH),
                robot.distanceL.getDistance(DistanceUnit.INCH),
                robot.yaw0));
        telemetry.update();
        waitForStart();

        int forwardTicks = 4000; // default 1050
        driveMotors(forwardTicks, forwardTicks, forwardTicks, forwardTicks, 0.6,
                true, robot.yaw0);

        while(opModeIsActive()) {
            telemetry.addLine(String.format("DistanceR: %.1f inch\nDistanceL: %.1f inch\nCurrent Yaw: %.1f",
                    robot.distanceR.getDistance(DistanceUnit.INCH),
                    robot.distanceL.getDistance(DistanceUnit.INCH),
                    robot.getCurrentYaw()));
            telemetry.update();
        }

        double distThreshold = 10.0;
        int spikeMode = 2;
        int ticks = 0;



        if (robot.distanceL.getDistance(DistanceUnit.INCH) < distThreshold) {
            spikeMode = 1;
            // Spike 1
            // back ~3 inches
            ticks = 200;
            requestOpModeStop();

            driveMotors(ticks, ticks, ticks, ticks, -0.6,
                    false, robot.yaw0);
            requestOpModeStop();

            // Turn left 90 degree
            turnToTargetYaw(robot.yaw0+90, 0.6, 3000);

            // Release preloaded pixel 1
            deployPreloadedPixel1(800);
            // Turn right 180 degree
            turnToTargetYaw(robot.yaw0-90, 0.8, 1000);

            // Move forward distance ~48 inches


        }
        else if (robot.distanceR.getDistance(DistanceUnit.INCH) < distThreshold){
            spikeMode = 3;

            // back about 6 inches

            // move right for about 5 inches

            // release preloaded pixel 1

            // turn right 90 degree

            // move forward

            // move left

            // orient

        }
        else {
            spikeMode = 2;
            // do nothing.


        }

        // Deploy preloaded pixel 1
        /*
        robot.preloader1.setPosition(1.0);
        sleep(1000);
        robot.preloader1.setPosition(0.5);
        sleep(1000);
        requestOpModeStop();

             */
    }

    private void driveMotors(int flTarget, int blTarget, int frTarget, int brTarget,
                             double power,
                             boolean bKeepYaw, double targetYaw){
        double currentYaw, diffYaw;
        double powerDeltaPct, powerL, powerR;
        double powerLeftPctToCounterCOG = 1.00;
        int direction;

        robot.motorfl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorbl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorfr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorbr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set Brake zero power behavior
        robot.motorfr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.motorfl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.motorbr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.motorbl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.motorfl.setTargetPosition((int)(flTarget * powerLeftPctToCounterCOG));
        robot.motorbl.setTargetPosition((int)(blTarget * powerLeftPctToCounterCOG));
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
        /*
        while(opModeIsActive() &&
                (robot.motorfl.isBusy() ||
                        robot.motorbl.isBusy() ||
                        robot.motorfr.isBusy() ||
                        robot.motorbr.isBusy())){

         */
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
                // Counter right sided COG by reducing power on the left.
                powerL *= powerLeftPctToCounterCOG;
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
            if (ticks > 50)
                ticks = 50;
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
