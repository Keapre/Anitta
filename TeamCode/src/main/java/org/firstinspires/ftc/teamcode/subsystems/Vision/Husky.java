package org.firstinspires.ftc.teamcode.subsystems.Vision;
import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.checkerframework.checker.units.qual.C;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@Config
public class Husky{

    private final int READ_PERIOD = 1;
    private final int FRAME_WIDTH = 320;
    private int LEFT_THRESHOLD = FRAME_WIDTH / 3;
    private int RIGHT_THRESHOLD = 2 * (FRAME_WIDTH / 3) - 20;

    public static int yThreshold = 70;


    private HuskyLens huskyLens;
    private HuskyLens.Block[] blocks;

    public Husky(HardwareMap hardwareMap) {
        huskyLens = hardwareMap.get(HuskyLens.class,"huskyLens");

        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        updateBlocks();
    }

    public void updateBlocks() {
        blocks = huskyLens.blocks();
    }

    public int getLocation(boolean red) {
        updateBlocks();

//        if(blocks.length == 0) return 1;
        int location = 0;
//        ArrayList<HuskyLens.Block> blocksx = new ArrayList<>();
//        ArrayList<Integer> blockWidth = new ArrayList<Integer>();
//        for( HuskyLens.Block blk : blocks) {
//            if((blk.id == 1 && red) || (blk.id == 2 && !red) && blk.y <= 160){
//                blocksx.add(blk);
//                blockWidth.add(blk.width);
//            }
//
//        }
//
//
//        if(blockWidth.size() == 0) return 1;
//        blockWidth.sort(Collections.reverseOrder());

        for(HuskyLens.Block blk : blocks) {
//               Log.w("y",Double.toString(blk.y));
//               Log.w("id",Double.toString(blk.id));
                if((blk.id == 1 && red) || (blk.id == 2 && !red) ) {
                    int x = blk.x;
                    int y = blk.y;
                    if(x<=LEFT_THRESHOLD) {
                        location = 0;
                    }else if(x <= RIGHT_THRESHOLD) {
                        location = 1;
                    }else {
                        location = 2;
                    }
                    break;
                }

        }
        return location;
    }
}