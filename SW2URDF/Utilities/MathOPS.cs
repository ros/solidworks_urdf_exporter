/*
Copyright (c) 2015 Stephen Brawner

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

using MathNet.Numerics.LinearAlgebra.Double;
using MathNet.Numerics.LinearAlgebra.Generic;
using SolidWorks.Interop.sldworks;
using System;
using System.Collections.Generic;

namespace SW2URDF.Utilities
{
    public static class MathOps
    {
        public static double epsilon = 1e-15;

        public static T Max<T>(T d1, T d2, T d3) where T : IComparable<T>
        {
            return Max(new T[] { d1, d2, d3 });
        }

        public static T Max<T>(T[] array) where T : IComparable<T>
        {
            T result = default(T);
            if (array.Length > 0)
            {
                result = array[0];
                foreach (T t in array)
                {
                    result = Comparer<T>.Default.Compare(t, result) > 0 ? t : result;
                }
            }
            return result;
        }

        public static T Min<T>(T d1, T d2, T d3) where T : IComparable<T>
        {
            return Min(new T[] { d1, d2, d3 });
        }

        public static T Min<T>(T[] array) where T : IComparable<T>
        {
            T result = default(T);
            if (array.Length > 0)
            {
                result = array[0];
                foreach (T t in array)
                {
                    result = Comparer<T>.Default.Compare(t, result) < 0 ? t : result;
                }
            }
            return result;
        }

        public static T Envelope<T>(T value, T min, T max) where T : IComparable<T>
        {
            if (Comparer<T>.Default.Compare(value, max) > 0)
            {
                return max;
            }
            else if (Comparer<T>.Default.Compare(value, min) < 0)
            {
                return min;
            }
            else
            {
                return value;
            }
        }

        public static double[] ClosestPointOnLineToPoint(double[] point, double[] line, double[] pointOnLine)
        {
            if (point.Length != line.Length || point.Length != pointOnLine.Length)
            {
                throw new Exception("Points and line vectors are not the same length");
            }

            double denominator = 0;
            double numerator = 0;
            for (int i = 0; i < point.Length; i++)
            {
                denominator += line[i] * line[i];
                numerator += line[i] * (point[i] - pointOnLine[i]);
            }
            double k = numerator / denominator;
            double[] result = new double[point.Length];
            for (int i = 0; i < result.Length; i++)
            {
                result[i] = pointOnLine[i] + k * line[i];
            }
            return result;
        }

        public static double[] ClosestPointOnLineWithinBox(
            double xMin, double xMax, double yMin, double yMax, double zMin, double zMax,
            double[] line, double[] pointOnLine)
        {
            if (pointOnLine[0] > xMin &&
                pointOnLine[0] < xMax &&
                pointOnLine[1] > yMin &&
                pointOnLine[1] < yMax &&
                pointOnLine[2] > zMin &&
                pointOnLine[2] < zMax)
            {
                return pointOnLine;
            }
            double[] point1 =
                ClosestPointOnLineToPoint(new double[] { xMax, yMax, zMax }, line, pointOnLine);
            double[] point2 =
                ClosestPointOnLineToPoint(new double[] { xMin, yMin, zMin }, line, pointOnLine);

            if (Distance2(pointOnLine, point1) < Distance2(pointOnLine, point2))
            {
                return point1;
            }
            else
            {
                return point2;
            }
        }

       public static double[] GetXYZ(Matrix<double> m)
        {
            double[] XYZ = new double[3];
            XYZ[0] = m[0, 3]; XYZ[1] = m[1, 3]; XYZ[2] = m[2, 3];
            return XYZ;
        }

        public static double[] GetXYZ(MathTransform transform)
        {
            double[] XYZ = new double[3];
            XYZ[0] = transform.ArrayData[9];
            XYZ[1] = transform.ArrayData[10];
            XYZ[2] = transform.ArrayData[11];
            return XYZ;
        }

        public static double[] GetRPY(Matrix<double> m)
        {
            double roll, pitch, yaw;
            double x, y;
            x = Math.Min(1.0, Math.Abs(m[2, 0])) * Math.Sign(m[2, 0]);
            y = Math.Min(1.0, Math.Abs(m[0, 2])) * Math.Sign(m[0, 2]);
            if (Math.Abs(m[2, 0]) >= 1.0)
            {
                // Gimbol Lock
                pitch = -Math.Asin(x);
                roll = Math.Acos(y);
                yaw = 0;
            }
            else
            {
                pitch = -Math.Asin(m[2, 0]);
                roll = Math.Atan2(m[2, 1], m[2, 2]);
                yaw = Math.Atan2(m[1, 0], m[0, 0]);
            }

            return new double[] { roll, pitch, yaw };
        }

        public static double[] GetRPY(MathTransform transform)
        {
            Matrix m = GetRotationMatrix(transform);
            return GetRPY(m);
        }

        public static Matrix<double> GetRotation(double[] RPY)
        {
            Matrix<double> RX = DenseMatrix.Identity(4);
            Matrix<double> RY = DenseMatrix.Identity(4);
            Matrix<double> RZ = DenseMatrix.Identity(4);

            RX[1, 1] = Math.Cos(RPY[0]);
            RX[1, 2] = -Math.Sin(RPY[0]);
            RX[2, 1] = Math.Sin(RPY[0]);
            RX[2, 2] = Math.Cos(RPY[0]);

            RY[0, 0] = Math.Cos(RPY[1]);
            RY[0, 2] = Math.Sin(RPY[1]);
            RY[2, 0] = -Math.Sin(RPY[1]);
            RY[2, 2] = Math.Cos(RPY[1]);

            RZ[0, 0] = Math.Cos(RPY[2]);
            RZ[0, 1] = -Math.Sin(RPY[2]);
            RZ[1, 0] = Math.Sin(RPY[2]);
            RZ[1, 1] = Math.Cos(RPY[2]);

            return RZ * RY * RX;
        }

        public static Matrix<double> GetTranslation(double[] XYZ)
        {
            Matrix<double> m = DenseMatrix.Identity(4);
            m[0, 3] = XYZ[0]; m[1, 3] = XYZ[1]; m[2, 3] = XYZ[2];
            return m;
        }

        public static Matrix<double> GetTransformation(double[] XYZ, double[] RPY)
        {
            Matrix<double> translation = GetTranslation(XYZ);
            Matrix<double> rotation = GetRotation(RPY);
            return translation * rotation;
        }

        public static Matrix GetRotationMatrix(MathTransform transform)
        {
            var rot = new DenseMatrix(3);
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    rot.At(i, j, transform.ArrayData[i + 3 * j]);
                }
            }

            return rot;
        }

        public static Matrix<double> GetTransformation(MathTransform transform)
        {
            Matrix<double> m = new DenseMatrix(4);

            m[0, 0] = transform.ArrayData[0];
            m[1, 0] = transform.ArrayData[1];
            m[2, 0] = transform.ArrayData[2];
            m[0, 1] = transform.ArrayData[3];
            m[1, 1] = transform.ArrayData[4];
            m[2, 1] = transform.ArrayData[5];
            m[0, 2] = transform.ArrayData[6];
            m[1, 2] = transform.ArrayData[7];
            m[2, 2] = transform.ArrayData[8];

            m[0, 3] = transform.ArrayData[9];
            m[1, 3] = transform.ArrayData[10];
            m[2, 3] = transform.ArrayData[11];
            m[3, 3] = transform.ArrayData[12];
            return m;
        }

        public static double[] PNorm(double[] array, double power)
        {
            double magnitude = 0;
            for (int i = 0; i < array.Length; i++)
            {
                magnitude += Math.Pow(array[i], power);
            }
            if (magnitude != 0)
            {
                magnitude = Math.Pow(magnitude, 1 / power);
                for (int i = 0; i < array.Length; i++)
                {
                    array[i] /= magnitude;
                }
            }
            return array;
        }

        public static double Distance2(double[] array1, double[] array2)
        {
            double sqrdmag = 0;
            for (int i = 0; i < array1.Length; i++)
            {
                double d = array1[i] - array2[i];
                sqrdmag += d * d;
            }
            return sqrdmag;
        }

        public static double[] Threshold(double[] array, double minValue)
        {
            double[] result = (double[])array.Clone();
            for (int i = 0; i < array.Length; i++)
            {
                result[i] = (Math.Abs(array[i]) >= minValue) ? array[i] : 0;
            }
            return result;
        }
    }
}