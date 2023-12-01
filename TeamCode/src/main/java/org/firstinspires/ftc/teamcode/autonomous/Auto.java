package org.firstinspires.ftc.teamcode.autonomous;
//test
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import android.graphics.drawable.GradientDrawable;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
//ignore this for now
@Autonomous(name="Auto")
public class Auto extends LinearOpMode {
    RobotHardware robot = new RobotHardware();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        //reset encoder
        robot.setAutoDriveMotorMode();

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addLine(String.format("DistanceR: %.1f inch\nDistanceL: %.1f inch\n",
                    robot.distanceR.getDistance(DistanceUnit.INCH),
                    robot.distanceL.getDistance(DistanceUnit.INCH)));
            telemetry.update();

            if (robot.distanceR.getDistance(DistanceUnit.INCH) < 48) {
                //robot.preloader1.setPosition(1.0);
                sleep(1000);
                //robot.preloader1.setPosition(0.5);
                sleep(1000);
                requestOpModeStop();
            }
        }
    }


}
