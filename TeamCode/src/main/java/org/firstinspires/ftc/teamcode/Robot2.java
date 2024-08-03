package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.util.Globals.START_LOOP;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.Drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Drive.WolfPackDrive;
import org.firstinspires.ftc.teamcode.subsystems.EndGame.Plane;
import org.firstinspires.ftc.teamcode.subsystems.Intake.Extendo;
import org.firstinspires.ftc.teamcode.subsystems.Intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake.Slides;
import org.firstinspires.ftc.teamcode.subsystems.Sensors;
import org.firstinspires.ftc.teamcode.util.Priority.HardwareQueue;

public class Robot2 {
    HardwareQueue hardwareQueue;

    public final MecanumDrive drivetrain;
    public final WolfPackDrive wolfPackDrive;

    public final Sensors sensors;

    public final Intake intake;

    public final Plane plane;

    public final Slides slides;

    public final Extendo extendo;

    public final Outtake outtake;

    public Robot2(HardwareMap hardwareMap) {
        hardwareQueue = new HardwareQueue();

        drivetrain = new MecanumDrive(hardwareMap,new Pose2d(0,0,0),hardwareQueue);
        wolfPackDrive = new WolfPackDrive(drivetrain);

        sensors = new Sensors(hardwareMap, hardwareQueue);

        intake = new Intake(hardwareMap, hardwareQueue,this);

        plane = new Plane(hardwareMap, hardwareQueue);

        slides = new Slides(hardwareMap, hardwareQueue,sensors,this);

        extendo = new Extendo(hardwareMap, hardwareQueue,sensors,this);

        outtake = new Outtake(hardwareMap, hardwareQueue,this);

    }

    public void update() {
        START_LOOP();
        updateSubsystems();
    }
    public void updateSubsystems() {
        intake.Update.start();
        plane.update();
        slides.update();
        extendo.update();
        outtake.update();
        drivetrain.updatePoseEstimate();
        hardwareQueue.update();
    }


}
