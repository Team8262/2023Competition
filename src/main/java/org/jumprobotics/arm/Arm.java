package org.jumprobotics.arm;


import edu.wpi.first.math.geometry.Translation2d;

public interface Arm {
    public double[][] toAngles(Translation2d target, Method method);
    public Translation2d toPosition(double[] angles);
    

    public enum Method{
        INVERSE_KINEMATICS,
        LOOKUP_TABLE
    }
}
