// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.jumprobotics.util;

import java.util.HashMap;

import org.json.simple.JSONObject;

import edu.wpi.first.math.geometry.Translation2d;


/** Add your docs here. */
public class LookupTable {
    private JSONObject table;

    public void readTable(String path){
        table = JSONReader.read(path);
    }

    public double[][] getAngles(Translation2d position){
        return (double[][]) table.get(stringify(position));
    }

    private String stringify(Translation2d position){
        return "(" + position.getX() + "," + position.getY() + ")";
    }

}
