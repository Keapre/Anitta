package org.firstinspires.ftc.teamcode.util.Priority;
import static org.firstinspires.ftc.teamcode.util.Globals.GET_LOOP_TIME;

import java.util.ArrayList;
import java.util.PriorityQueue;

import org.firstinspires.ftc.teamcode.util.Globals;

import java.util.Comparator;
class Comp implements Comparator<PriorityDevice>  {


    @Override
    public int compare(PriorityDevice priorityDevice, PriorityDevice t1) {
        if(priorityDevice.getPriority(0.010 - GET_LOOP_TIME()) < t1.getPriority(0.010 - GET_LOOP_TIME())) {
            return 1;
        }else {
            return 0;
        }
    }

}
public class HardwareQueue {
    public ArrayList<PriorityDevice> devices = new ArrayList<>();
    public double targetLoopLength = 0.010;

    public PriorityDevice getDevice(String name){
        for (PriorityDevice device : devices){
            if (device.name.equals(name)){
                return device;
            }
        }
        return null;
    }

    public void addDevice(PriorityDevice device) {
        devices.add(device);
    }
    public void update() {
        PriorityQueue<PriorityDevice> pq = new PriorityQueue<PriorityDevice>(new Comp());
        for(PriorityDevice device : devices) {
            device.resetUpdateBoolean();
            pq.add(device);
        }

        while(!pq.isEmpty()) {
            pq.poll().update();
        }
    }
}
