package com.android.ball_balancing_system;

public class Matrix {
    private final double[][] data;
    private final int rows;
    private final int cols;

    public Matrix(double[][] data) {
        this.data = data;
        this.rows = data.length;
        this.cols = data[0].length;
    }

    public static Matrix identity(int size) {
        double[][] I = new double[size][size];
        for (int i = 0; i < size; i++) {
            I[i][i] = 1;
        }
        return new Matrix(I);
    }

    // Matrix multiplication
    public Matrix times(Matrix other) {
        if (this.cols != other.rows) {
            throw new IllegalArgumentException("Matrix dimensions don't match for multiplication");
        }

        double[][] result = new double[rows][other.cols];
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < other.cols; j++) {
                for (int k = 0; k < cols; k++) {
                    result[i][j] += data[i][k] * other.data[k][j];
                }
            }
        }
        return new Matrix(result);
    }

    // Scalar multiplication
    public Matrix times(double scalar) {
        double[][] result = new double[rows][cols];
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                result[i][j] = data[i][j] * scalar;
            }
        }
        return new Matrix(result);
    }

    // Matrix addition
    public Matrix plus(Matrix other) {
        if (this.rows != other.rows || this.cols != other.cols) {
            throw new IllegalArgumentException("Matrix dimensions don't match for addition");
        }

        double[][] result = new double[rows][cols];
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                result[i][j] = data[i][j] + other.data[i][j];
            }
        }
        return new Matrix(result);
    }

    // Matrix subtraction
    public Matrix minus(Matrix other) {
        if (this.rows != other.rows || this.cols != other.cols) {
            throw new IllegalArgumentException("Matrix dimensions don't match for subtraction");
        }

        double[][] result = new double[rows][cols];
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                result[i][j] = data[i][j] - other.data[i][j];
            }
        }
        return new Matrix(result);
    }

    // Matrix transpose
    public Matrix transpose() {
        double[][] result = new double[cols][rows];
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                result[j][i] = data[i][j];
            }
        }
        return new Matrix(result);
    }

    // Matrix inverse (using Gaussian elimination)
    public Matrix inverse() {
        if (rows != cols) {
            throw new IllegalArgumentException("Matrix must be square to invert");
        }

        int n = rows;
        double[][] inverse = new double[n][n];

        // Initialize inverse as identity matrix
        for (int i = 0; i < n; i++) {
            inverse[i][i] = 1;
        }

        // Create a copy of the original matrix
        double[][] a = new double[n][n];
        for (int i = 0; i < n; i++) {
            System.arraycopy(data[i], 0, a[i], 0, n);
        }

        // Perform Gaussian elimination
        for (int k = 0; k < n; k++) {
            // Find the pivot row
            int max = k;
            for (int i = k + 1; i < n; i++) {
                if (Math.abs(a[i][k]) > Math.abs(a[max][k])) {
                    max = i;
                }
            }

            // Swap rows
            double[] temp = a[k];
            a[k] = a[max];
            a[max] = temp;

            double[] tempInv = inverse[k];
            inverse[k] = inverse[max];
            inverse[max] = tempInv;

            // Singular matrix check
            if (Math.abs(a[k][k]) < 1e-10) {
                throw new RuntimeException("Matrix is singular or nearly singular");
            }

            // Eliminate column
            for (int i = 0; i < n; i++) {
                if (i != k) {
                    double factor = a[i][k] / a[k][k];
                    for (int j = k; j < n; j++) {
                        a[i][j] -= a[k][j] * factor;
                    }
                    for (int j = 0; j < n; j++) {
                        inverse[i][j] -= inverse[k][j] * factor;
                    }
                }
            }
        }

        // Normalize diagonal elements
        for (int i = 0; i < n; i++) {
            double factor = a[i][i];
            for (int j = 0; j < n; j++) {
                inverse[i][j] /= factor;
            }
        }

        return new Matrix(inverse);
    }

    // Get element at specific position
    public double get(int row, int col) {
        return data[row][col];
    }
    public void set(int row, int col, double value) {
        if (row < 0 || row >= rows || col < 0 || col >= cols) {
            throw new IndexOutOfBoundsException(
                    String.format("Index (%d,%d) out of bounds for %dx%d matrix",
                            row, col, rows, cols));
        }
        data[row][col] = value;
    }
}
