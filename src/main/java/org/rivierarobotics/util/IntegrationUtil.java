/*
 * This file is part of CheeseWrangler-2020, licensed under the GNU General Public License (GPLv3).
 *
 * Copyright (c) Riviera Robotics <https://github.com/Team5818>
 * Copyright (c) contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

package org.rivierarobotics.util;

import java.util.function.Function;

public class IntegrationUtil {
    public static final IntegrationUtil DEFAULT = new IntegrationUtil(10);
    private final double[] roots;
    private final double[] weights;
    private final double[][] coeff;
    private final int n;

    /**
     * Numerical integration helper class using Gauss-Legendre Quadrature.
     * @param n The number of points to evaluate with
     */
    public IntegrationUtil(int n) {
        this.roots = new double[n];
        this.weights = new double[n];
        this.coeff = new double[n + 1][n + 1];
        this.n = n;
        calcCoeff();
        calcRoots();
    }

    private void calcCoeff() {
        coeff[0][0] = coeff[1][1] = 1;
        for (int i = 2; i <= n; i++) {
            coeff[i][0] = -(i - 1) * coeff[i - 2][0] / i;
            for (int j = 1; j <= i; j++) {
                coeff[i][j] = ((2 * i - 1) * coeff[i - 1][j - 1]
                        - (i - 1) * coeff[i - 2][j]) / i;
            }
        }
    }

    private void calcRoots() {
        double x;
        double x1;
        for (int i = 1; i <= n; i++) {
            x = Math.cos(Math.PI * (i - 0.25) / (n + 0.5));
            do {
                x1 = x;
                x -= eval(x) / diff(x);
            } while (x != x1);
            roots[i - 1] = x;
            x1 = diff(x);
            weights[i - 1] = 2 / ((1 - x * x) * x1 * x1);
        }
    }

    private double eval(double x) {
        return eval(n, x);
    }

    private double eval(int k, double x) {
        double s = coeff[k][k];
        for (int i = k; i > 0; i--) {
            s = s * x + coeff[k][i - 1];
        }
        return s;
    }

    private double diff(double x) {
        return n * (x * eval(n, x) - eval(n - 1, x)) / (x * x - 1);
    }

    /**
     * Integrates a function numerically using Gauss-Legendre Quadrature.
     * @param fn The function to be integrated
     * @param a Lower bound
     * @param b Upper bound
     * @return The integrated function fn from a to b
     */
    public double integrate(Function<Double, Double> fn, double a, double b) {
        double c1 = (b - a) / 2;
        double c2 = (b + a) / 2;
        double sum = 0;
        for (int i = 0; i < n; i++) {
            sum += weights[i] * fn.apply(c1 * roots[i] + c2);
        }
        return c1 * sum;
    }
}
