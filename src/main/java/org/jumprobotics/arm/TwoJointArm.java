// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.jumprobotics.arm;

import org.jumprobotics.math.Polynomial;

import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public class TwoJointArm implements Arm{
    
    private Polynomial baseLink;

    public TwoJointArm(double length1, double length2) {

    }

    public double[] toAngles(Translation2d position) {
        return null;
    }

    public Translation2d toPosition(double[] angles) {
        return new Translation2d();
    }
    

}
