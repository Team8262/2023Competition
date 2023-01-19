package org.jumprobotics.math;

import edu.wpi.first.math.geometry.Translation2d;

public class Polynomial {
    private double[] coefficients;

    public Polynomial(double[] coefficients) {
        this.coefficients = coefficients;
    }

    public double evaluate(double x) {
        double result = 0;
        for (int i = 0; i < coefficients.length; i++) {
            result += coefficients[i] * Math.pow(x, i);
        }
        return result;
    }

    public double[] getCoefficients() {
        return coefficients;
    }

    /*
     * Return the roots of the polynomial
     * Only works for polynomials of degree 2 or less
     */
    public double[] getRoots() {
        if (coefficients.length == 2) {
            return new double[]{-coefficients[0] / coefficients[1]};
        } else if (coefficients.length == 3) {
            double a = coefficients[2];
            double b = coefficients[1];
            double c = coefficients[0];
            double discriminant = b * b - 4 * a * c;
            if (discriminant < 0) {
                return new double[0];
            } else if (discriminant == 0) {
                return new double[]{-b / (2 * a)};
            } else {
                return new double[]{(-b + Math.sqrt(discriminant)) / (2 * a), (-b - Math.sqrt(discriminant)) / (2 * a)};
            }
        } else {
            throw new IllegalArgumentException("Polynomial must be of degree 2 or less");
        }
    }

    public Translation2d[] intersect(Polynomial other){
        double[] otherCoefficients = other.getCoefficients();
        double[] coefficients = new double[3];
        coefficients[0] = this.coefficients[0] - otherCoefficients[0];
        coefficients[1] = this.coefficients[1] - otherCoefficients[1];
        coefficients[2] = this.coefficients[2] - otherCoefficients[2];
        Polynomial p = new Polynomial(coefficients);
        double[] roots = p.getRoots();
        Translation2d[] intersections = new Translation2d[roots.length];
        for (int i = 0; i < roots.length; i++) {
            intersections[i] = new Translation2d(roots[i], evaluate(roots[i]));
        }
        return intersections;
    }

}
