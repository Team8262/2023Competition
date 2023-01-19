// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.jumprobotics.math;

import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public class Circle {
    private Translation2d center;
    private double radius;
    public Circle(Translation2d center, double radius) {
        this.center = center;
        this.radius = radius;
    }

    public boolean intersects(Circle other) {
        return center.getDistance(other.getCenter()) < radius + other.getRadius();
    }

    public Translation2d[] intersections(Circle other){
        if(!intersects(other)) return null;
        double d = center.getDistance(other.getCenter());
        double a = (radius*radius - other.getRadius()*other.getRadius() + d*d)/(2*d);
        double h = Math.sqrt(radius*radius - a*a);
        Translation2d p2 = center.interpolate(other.getCenter(), a/d);
        Translation2d p3 = new Translation2d(p2.getX() + h*(other.getCenter().getY() - center.getY())/d, p2.getY() - h*(other.getCenter().getX() - center.getX())/d);
        Translation2d p4 = new Translation2d(p2.getX() - h*(other.getCenter().getY() - center.getY())/d, p2.getY() + h*(other.getCenter().getX() - center.getX())/d);
        return new Translation2d[]{p3, p4};
    }

    public double getRadius() {
        return radius;
    }

    public Translation2d getCenter() {
        return center;
    }

}
