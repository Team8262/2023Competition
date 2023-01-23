package org.jumprobotics.arm;

import edu.wpi.first.math.geometry.Translation2d;

public interface Arm {
    public double[][] toAngles(Translation2d target);
    public Translation2d toPosition(double[] angles);
    
}
