// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.jumprobotics.util;

import org.json.simple.JSONObject;

import com.team254.lib.util.InterpolatingDouble;
import com.team254.lib.util.InterpolatingTreeMap;

/** Add your docs here. */
public class LookupTable {
    private InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> table;

    private void readTable(String path){
        JSONObject json = JSONReader.read(path);
    }

}
