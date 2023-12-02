package org.firstinspires.ftc.teamcode.autonomous;
//test
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
//ignore this for now
@Autonomous(name="z_Unused_Auto")
@Disabled
public class z_Unused_Auto extends LinearOpMode {
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
