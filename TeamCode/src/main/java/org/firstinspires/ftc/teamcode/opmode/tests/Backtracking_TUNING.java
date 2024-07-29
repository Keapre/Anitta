package org.firstinspires.ftc.teamcode.opmode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.rr.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.Priority.HardwareQueue;

@Config
@TeleOp(name = "Backtracking_Tuning", group = "Linear OpMode")
public class Backtracking_TUNING extends LinearOpMode {
    //You can tune the time at which the program reads the IMU to find best accuracy
    public static int timeBetween_Reads = 300;
    MecanumDrive drive;
    ElapsedTime loopTime = new ElapsedTime();
    double total = 0.0, NumOfLoops =0.0;
    HardwareQueue hardwareQueue;
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        hardwareQueue = new HardwareQueue();

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0),hardwareQueue);

        loopTime.reset();

        while(opModeInInit()) {
            telemetry.addLine("The DIRECTIONS");
            telemetry.addLine("Tune variable timeBetween_Reads in the FTC Dashboard");
            telemetry.addLine("According to the loopTimes and Effectiveness of the Backtracking");
            telemetry.addLine("Turn and push your robot for about 30 seconds in every direction");
            telemetry.update();
        }

        telemetry.clearAll();
        waitForStart();
        while (opModeIsActive()) {

            drive.updatePoseEstimate();

            NumOfLoops ++;

            total += loopTime.milliseconds();
            telemetry.addData("LoopTime", (int)loopTime.milliseconds());
            if(NumOfLoops!=0){
                telemetry.addData("Avg LoopT - Restart Op Mode after Changing timeBetween_Reads)", total/NumOfLoops);
            }
            loopTime.reset();

            //Weird X is Y things are because Y in RR is lateral movement
            telemetry.addData("Overall Effectiveness X - In Percentage", drive.totalMovement()[1]);
            telemetry.addData("Overall Effectiveness Y - In Percentage", drive.totalMovement()[0]);

            telemetry.addData("Error recognized in degrees (R)", drive.OverallError[2]);
            telemetry.addData("Error recognized in inches (X)", drive.OverallError[1]);
            telemetry.addData("Error recognized in inches (Y)", drive.OverallError[0]);

            telemetry.addData("Rotation - Pose", Math.toDegrees(drive.pose.heading.toDouble()));
            telemetry.addData("X - Pose", drive.pose.position.y);
            telemetry.addData("Y - Pose", drive.pose.position.x);
            telemetry.update();
        }

    }


}