package org.firstinspires.ftc.teamcode.subsystems.EndGame;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Caching.CachingServo;
import org.firstinspires.ftc.teamcode.util.Priority.HardwareQueue;
import org.firstinspires.ftc.teamcode.util.Priority.PriorityDevice;
import org.firstinspires.ftc.teamcode.util.Priority.PriorityServo;

public class Plane {
    public enum status {
        IN,
        OUT
    }
    PriorityServo plane;
    public status Status;
    double powerPosition = 0.69; //TODO:change this value

    double[] positions = new double[]{0.10,0.69};
    double close = 0.10;
    public Plane(HardwareMap hardwareMap, HardwareQueue hardwareQueue){
        this.plane = new PriorityServo(
                new CachingServo(hardwareMap.get(Servo.class, "plane")),
                "plane",
                3,
                PriorityServo.ServoType.AXON_MINI,
                positions[0],
                positions[0],
                false
        );
        this.plane.setPosition(positions[0]);
        this.Status = status.IN;
        hardwareQueue.addDevice(plane);
    }


    public void setOut() {
        this.Status = status.OUT;
    }
    public void update() {
        switch (Status) {
            case IN:
                this.plane.setPosition(positions[0]);
                break;
            case OUT:
                this.plane.setPosition(positions[1]);
                break;

        }
    }

}
