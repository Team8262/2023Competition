// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.jumprobotics.util;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;

import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

import edu.wpi.first.wpilibj.Filesystem;

/** Add your docs here. */
public class JSONReader {

    public static JSONObject read(String path) {
        try (BufferedReader br =
        new BufferedReader(
            new FileReader(
                new File(Filesystem.getDeployDirectory(), path)))) {
        StringBuilder fileContentBuilder = new StringBuilder();
        String line;
        while ((line = br.readLine()) != null) {
            fileContentBuilder.append(line);
        }

        String fileContent = fileContentBuilder.toString();
        JSONObject json = (JSONObject) new JSONParser().parse(fileContent);
        return json;

        } catch (Exception e) {
            e.printStackTrace();
            return null;
        }
    }

}
