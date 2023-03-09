// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.jumprobotics.arm;


import org.jumprobotics.util.LookupTable;

import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public class TwoJointArm implements Arm{
    
    private double length1, length2;
    private LookupTable table;

    public TwoJointArm(double length1, double length2) {
        this.length1 = length1;
        this.length2 = length2;
        table = new LookupTable();
    }

    public void addLookupTable(String path){
        table.readTable(path);
    }

    public double[][] toAngles(Translation2d position, Method method) {
        switch(method){
            case INVERSE_KINEMATICS:
                return inverseKinematics(position);
            case LOOKUP_TABLE:
                return lookupTable(position);
            default:
                return inverseKinematics(position);
        }

    }

    public boolean isValid(Translation2d position) {
        return (position.getX() * position.getX()) + (position.getY() * position.getY()) <= (length1 + length2) * (length1 + length2);
    }

    private double[][] lookupTable(Translation2d position) {
        return table.getAngles(position);
    }
    //Presumably the independent angle is upper while the dependent is base
    private double[][] inverseKinematics(Translation2d position) {
        double x = position.getX();
        double y = position.getY();
        double angle2 = Math.acos((x*x + y*y - length1*length1 - length2*length2)/(2*length1*length2));
        double angle1 = Math.atan(y/x) - Math.atan(length2*Math.sin(angle2)/(length1+length2*Math.cos(angle2)));
        double[] solution1 = {angle1, angle2};
        double angle3 = -Math.acos((x*x + y*y - length1*length1 - length2*length2)/(2*length1*length2));
        double angle4 = Math.atan(y/x) + Math.atan(length2*Math.sin(angle3)/(length1+length2*Math.cos(angle3)));
        double[] solution2 = {angle4, angle3};
        return new double[][]{solution1, solution2};
    }

    //This doesn't seem right - should check later
    public Translation2d toPosition(double[] angles) {
        double x = length1 * Math.cos(angles[0]) + length2 * Math.cos(angles[0] + angles[1]);
        double y = length1 * Math.sin(angles[0]) + length2 * Math.sin(angles[0] + angles[1]);
        return new Translation2d(x, y);
    }



}
