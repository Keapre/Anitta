package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.util.Globals.START_LOOP;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.rr.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Drive.WolfPackDrive;
import org.firstinspires.ftc.teamcode.util.Priority.HardwareQueue;

public class Robot {
    HardwareQueue hardwareQueue;

    public MecanumDrive drivetrain;
    public WolfPackDrive wolfPackDrive;

    public Robot(HardwareMap hardwareMap) {
        hardwareQueue = new HardwareQueue();

        drivetrain = new MecanumDrive(hardwareMap,new Pose2d(0,0,0),hardwareQueue);
        wolfPackDrive = new WolfPackDrive(drivetrain);
    }

    public void update() {
        START_LOOP();
        updateSubsystems();
    }
    public void updateSubsystems() {
        hardwareQueue.update();
    }
}
