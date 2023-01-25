// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team254.lib.util;

import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public class InterpolatingTranslation2d implements Interpolable<InterpolatingTranslation2d>, InverseInterpolable<InterpolatingTranslation2d{

    private Translation2d translation;

    public InterpolatingTranslation2d(Translation2d translation) {
        this.translation = translation;
    }

    public InterpolatingTranslation2d(double x, double y) {
        this.translation = new Translation2d(x, y);
    }

    @Override
    public InterpolatingTranslation2d interpolate(InterpolatingTranslation2d other, double x) {
        Translation2d interpolated = translation.interpolate(other.translation, x);
        return new InterpolatingTranslation2d(interpolated);
    }

    @Override
    public double inverseInterpolate(InterpolatingTranslation2d other, InterpolatingTranslation2d value) {
        return translation.inverseInterpolate(other.translation, value.translation);
    }

    public Translation2d getTranslation() {
        return translation;
    }
}
