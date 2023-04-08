package org.jumprobotics.gyro;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;

public class GyroIONavX implements GyroIO{

    private final AHRS gyro;

    public GyroIONavX(){
        gyro = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX connected over MXP
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
      inputs.connected =gyro.isConnected();
      inputs.positionDeg = gyro.getYaw(); // degrees
      inputs.velocityDegPerSec = -gyro.getRate(); //xyzDps[2]  degrees per second
    }
    
}
