package org.firstinspires.ftc.teamcode;

import android.app.Application;
import android.content.Context;
import android.view.View;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;


import org.firstinspires.ftc.robotcore.internal.system.AppUtil;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.HashMap;

/**
 * Created by tycho on 2/18/2017. This was copied from Team FIXIT. All hail team FIXIT.
 * This is simply a set of utils to reference the main robot controller activity for convenience
 */

public class RC {
    public static OpMode o;
    public static LinearOpMode l;

    public static HardwareMap h;
    public static int runNum = 0;
    public final static String VUFORIA_LICENSE_KEY = "ATL6i4D/////AAAAGTkmmlOAlE6bi5wxY+GFDEYkiReJ4JoLKVqIj7L5JPEFanFxDXGWvNPh5QR4YboR1fVEnH7msYfNfuIiuARyXfZFlOWBYQ7PYL7s6zhTc7dDhxeF/HKTiiNUsnS2ahWhMbOyOQcwERpRwjTOONg1gcObtCJWVHHui3EphLOLOUFsumD5jQ4V7SkAnQ8MpIOHM8ntNJ0cwcO2TYGWp7rhYwaX5pywNiFcTHn4/9QVSLjnWnA+iN8jypntxolwNAsYILKDTS+8vRgbOnNQWK/RTkEHDZxV0We627e/AINsgGQzwKb0cITEhnQDStnZJPAxv6mVY4+ssmng7ezD/M+JkiqrZhmtlk5mHx5+MTC+anMs";


    public static void setOpMode(OpMode op) {
        o = op;
        h = op.hardwareMap;


        if (op instanceof LinearOpMode) {
            l = (LinearOpMode) op;
        }//if

    }//setOpMode




    public static Context c() {
        return AppUtil.getInstance().getActivity();
    }//context

    public static FtcRobotControllerActivity a() {
        return ((FtcRobotControllerActivity) AppUtil.getInstance().getActivity());
    }//activity





}