using System;
using System.IO;
using System.Runtime.InteropServices;
using System.Collections;
using System.Reflection;

using System.Collections.Generic;
using System.Diagnostics;
using System.Windows.Forms;
using MathNet.Numerics.LinearAlgebra.Generic;
using MathNet.Numerics.LinearAlgebra.Double;
using MathNet.Numerics.LinearAlgebra;
using System.Numerics;

using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.swpublished;
using SolidWorks.Interop.swconst;
using SolidWorksTools;
using SolidWorksTools.File;

namespace SW2URDF
{
    public class ops
    {
        public double epsilon = 1e-15;
        // Convert a MATLAB type string representation of a matrix into a math.net numerics Matrix. Convenient for reading from text files
        public Matrix str2mat(string S)
        {
            
            S = S.Trim(new char[] { '[', ']', ' ' });
            string[] rows = S.Split(';');
            int rowCount = rows.Length;
            string[] firstRow = rows[0].Trim().Split(' ');
            int columnCount = firstRow.Length;
            Matrix m = new DenseMatrix(rowCount, columnCount);
            for (int i = 0; i < rowCount; i++)
            {
                rows[i] = rows[i].Trim();
                string[] columns = rows[i].Split(' ');
                if (columns.Length == columnCount)
                {
                    double value;
                    for (int j = 0; j < columnCount; j++)
                    {

                        m[i, j] = (Double.TryParse(columns[j], out value)) ? value : 0;
                    }
                }
            }
            return m;
        }

        // Convert a math.net Numerics Matrix into a MATLAB type string representation
        public string mat2str(Matrix m)
        {
            string s = "";
            for (int i = 0; i < m.RowCount; i++)
            {
                for (int j = 0; j < m.ColumnCount; j++)
                {
                    s = s.Insert(s.Length, m[i, j].ToString());
                    if (j != m.ColumnCount - 1)
                    {
                        s = s.Insert(s.Length, " ");
                    }
                }
                s = s.Insert(s.Length, "\n");
            }
            return s;
        }

        // Concatenates two vectors together. Why doesn't math.net numerics have this functionality
        public Vector vectorCat(Vector v1, Vector v2)
        {
            Vector vec = new DenseVector(v1.Count + v2.Count);
            v1.CopyTo(vec, 0, 0, v1.Count);
            v2.CopyTo(vec, 0, v1.Count, v2.Count);
            return vec;
        }
        public Vector vectorCat(Vector v1, Vector v2, Vector v3, Vector v4)
        {
            Vector vec = new DenseVector(v1.Count + v2.Count + v3.Count + v4.Count);
            v1.CopyTo(vec, 0, 0, v1.Count);
            v2.CopyTo(vec, 0, v1.Count, v2.Count);
            v3.CopyTo(vec, 0, v1.Count + v2.Count, v3.Count);
            v4.CopyTo(vec, 0, v1.Count + v2.Count + v3.Count, v4.Count);
            return vec;
        }

        // Calculates the row reduced echelon form of a matrix. It really sucks that math.net numerics doesn't include this as a builtin function
        // 6/28. Now I know why... It's difficult to write one that can do it with floating point precision. So this one currently is broken

        //public Matrix rref(Matrix m)
        //{
        //    int minDimension = Math.Min(m.ColumnCount, m.RowCount);
        //    // Swap rows to get prepared for row echelon form
        //    for (int i = 0; i < minDimension; i++)
        //    {
        //        for (int j = i; j < minDimension; j++)
        //        {
        //            Vector v = (Vector)m.Row(j);
        //            if (v[i] > epsilon)
        //            {
        //                m.SetRow(j, m.Row(i));
        //                m.SetRow(i, v);
        //                break;
        //            }
        //            else
        //            {
        //                m[j, i] = 0;
        //            }
        //        }
        //    }
        //    for (int i = 0; i < m.RowCount; i++)
        //    {
        //        Vector v1 = (Vector)m.Row(i);
        //        int index = -1;

        //        //Find the first non-zero entry in row i (will either by the diagonal or to its right)
        //        for (int j = i; j < m.ColumnCount; j++)
        //        {
        //            if (v1[j] > epsilon)
        //            {
        //                index = j;
        //                break;
        //            }
        //            else
        //            {
        //                m[i, j] = 0;
        //            }
        //        }

        //        //If there are no more non-zero entries to be found, the matrix is now in row echelon form (and then some)
        //        if (index == -1)
        //        {
        //            break;
        //        }
        //        for (int j = 0; j < m.RowCount; j++)
        //        {
        //            if (i != j)
        //            {

        //                Vector v2 = (Vector)m.Row(j);
        //                m.SetRow(j, v1 * v2[index] / v1[index] - v2);
        //            }
        //        }
        //    }

        //    //Reduce the left most values to 1
        //    for (int i = 0; i < m.RowCount; i++)
        //    {
        //        Vector v = (Vector)m.Row(i);
        //        int index = -1;
        //        //Find the first non-zero entry in this row vector
        //        for (int j = i; j < m.ColumnCount; j++)
        //        {
        //            if (v[j] > epsilon)
        //            {
        //                index = j;
        //                break;
        //            }
        //            else
        //            {
        //                v[j] = 0;
        //            }
        //        }
        //        //If there are no more non-zero entries to be found, the matrix is now in reduced row echelon form
        //        if (index == -1)
        //        {
        //            break;
        //        }
        //        //We are dividing by 0 here. Stop doing it!
        //        if (v[index] == 0)
        //        {
        //            int c = 1; //Whoops!
        //        }
        //        m.SetRow(i, v / v[index]);
        //    }
        //    return m;
        //}

        // Calculates the null space of a matrix
        //public Matrix nullSpace(Matrix m)
        //{
        //    m = rref(m); //Null(A) = Null(rref(A))
        //    Matrix null_m = new DenseMatrix(m.ColumnCount, m.RowCount);

        //    int lead = 0;
        //    Vector zeros = new DenseVector(m.RowCount);
        //    Vector column = new DenseVector(m.RowCount);
        //    for (int i = 0; i < m.ColumnCount; i++)
        //    {
        //        // This DOF is constrained
        //        if (m[i, lead] == 1)
        //        {
        //            null_m.SetColumn(i, zeros);
        //            lead++;
        //        }
        //        else
        //        {
        //            // Fill column vector with parameters
        //            for (int j = 0; j < lead; j++)
        //            {
        //                int columnIndex = findLeadingOneinVector((DenseVector)m.Row(j), 0, i - 1);
        //                if (columnIndex != -1)
        //                {
        //                    column[columnIndex] = m[j, i];
        //                }
        //                //else
        //                //{
        //                //    column[columnIndex] = 0;
        //                //}
        //            }
        //            null_m.SetColumn(i, column);
        //        }
        //    }
        //    return null_m;
        //}

        // This set of methods finds the bottom-most one in a column vector from a matrix.


        public int findLeadingOneinVector(Vector v)
        {
            return findLeadingOneinVector(v, 0, v.Count);
        }
        // Sets a lower bound in case this vector only has values thare are to the right of other leading ones
        public int findLeadingOneinVector(Vector v, int lowerBound)
        {
            return findLeadingOneinVector(v, lowerBound, v.Count);
        }
        // Sets an upper bound to reduce the number of computations. I.E in a rref matrix the 1 should be on or above the diagonal
        public int findLeadingOneinVector(Vector v, int lowerBound, int upperBound)
        {
            // If the upperBound is less than the lowerBound, the vector is searched backwards (to help speed up computation in some cases)
            if (upperBound < lowerBound)
            {
                for (int i = upperBound - 1; i >= lowerBound; i--)
                {
                    if (v[i] == 1)
                    {
                        return i;
                    }
                }
                return -1;
            }
            else
            {
                for (int i = lowerBound; i < upperBound; i++)
                {
                    if (v[i] == 1)
                    {
                        return i;
                    }
                }
                return -1;
            }
        }

        // Inserts the vector into the first empty row of a matrix (if the column count matches)
        public Matrix addVectorToMatrix(Matrix m, Vector v)
        {
            if (m.ColumnCount == v.Count)
            {
                int row = firstEmptyRow(m);
                if (row != -1)
                {
                    m.SetRow(row, v);
                }
            }
            return m;
        }

        // Finds the first empty (all entries are 0) row in a matrix. Returns -1 if no row is non-zero
        public int firstEmptyRow(Matrix m)
        {
            for (int i = 0; i < m.RowCount; i++)
            {
                bool isEmpty = true;
                for (int j = 0; j < m.ColumnCount; j++)
                {
                    if (m[i, j] != 0)
                    {
                        isEmpty = false;
                        break;
                    }
                }
                if (isEmpty)
                {
                    return i;
                }

            }
            return -1;
        }

        public Vector<double> projectLineToPlane(Vector<double> normal, Vector<double> line)
        {
            return crossProduct3(normal, crossProduct3(line, normal));

        }

        public Vector<double> crossProduct3(Vector<double> v1, Vector<double> v2)
        {
            Vector<double> v = new DenseVector(v1.Count);
            if (v1.Count == 3 && v2.Count == 3)
            {
                v[0] = v1[1] * v2[2] - v1[2] * v2[1];
                v[1] = v1[2] * v2[0] - v1[0] * v2[2];
                v[2] = v1[0] * v2[1] - v1[1] * v2[0];
            }
            return v;

        }

        public Matrix eig(Matrix<double> m)
        {
            if (m == null)
            {
                return null;
            }
            var eigen = m.Evd();
            var eigenValues = eigen.EigenValues();
            var eigenVectors = eigen.EigenVectors();
            var n = new DenseMatrix(eigenVectors.RowCount, eigenVectors.ColumnCount + 1);
            n.SetSubMatrix(0, eigenVectors.RowCount, 0, eigenVectors.ColumnCount, eigenVectors);
            for (int i = 0; i < eigenValues.Count; i++)
            {
                n.At(i, eigenVectors.ColumnCount, eigenValues[i].Magnitude);
            }
            return n;
        }

        public bool equals(Matrix<double> m1, Matrix<double> m2)
        {
            return equals(m1, m2, Double.Epsilon);

        }
        public bool equals(Matrix<double> m1, Matrix<double> m2, double epsilon)
        {
            if (m1.RowCount != m2.RowCount)
            {
                return false;
            }
            if (m1.ColumnCount != m2.ColumnCount)
            {
                return false;
            }
            for (int i = 0; i < m1.RowCount; i++)
            {
                for (int j = 0; j < m1.ColumnCount; j++)
                {
                    if (Math.Abs(m1[i, j] - m2[i, j]) >= epsilon)
                    {
                        return false;
                    }
                }
            }
            return true;
        }
        public bool equals(Vector<double> v1, Vector<double> v2)
        {
            return equals(v1, v2, Double.Epsilon);
        }
        public bool equals(Vector<double> v1, Vector<double> v2, double epsilon)
        {
            if (v1.Count != v2.Count)
            {
                return false;
            }
            for (int i = 0; i < v1.Count; i++)
            {
                if (Math.Abs(v1[i] - v2[i]) >= epsilon)
                {
                    return false;
                }
            }
            return true;
        }

        public int findDominantDirection(Matrix<double> m, double factor)
        {
            if (m.Rank() == 0)
            {
                // -2 for completely constrained
                return -2;
            }
            double[] zeros3 = new double[] { 0, 0, 0 };
            Vector<double> v1 = crossProduct3(m.Row(0)/m.Row(0).Norm(2), m.Row(1)/m.Row(1).Norm(2));
            if (v1.Norm(2) < 0.1)
            {
                m.SetRow(1, zeros3);
            }
            Vector<double> v2 = crossProduct3(m.Row(0) / m.Row(0).Norm(2), m.Row(2) / m.Row(2).Norm(2));
            if (v2.Norm(2) < 0.1)
            {
                m.SetRow(2, zeros3);
            }
            Vector<double> v3 = crossProduct3(m.Row(1) / m.Row(1).Norm(2), m.Row(2) / m.Row(2).Norm(2));
            if (v3.Norm(2) < 0.1)
            {
                if (m.Row(1).Norm(2) > m.Row(2).Norm(2))
                {
                    m.SetRow(2, zeros3);
                }
                else
                {
                    m.SetRow(1, zeros3);
                }
            }
            
            double nextMagnitude = 0;
            double largestMagnitude = 0;
            int selectedRow = -1;
            for (int i = 0; i < m.RowCount; i++)
            {
                Vector<double> v = m.Row(i);
                double mag = v.Norm(2);
                if (mag > largestMagnitude)
                {
                    nextMagnitude = largestMagnitude;
                    largestMagnitude = mag;
                    selectedRow = i;
                }
                else if (mag > nextMagnitude)
                {
                    nextMagnitude = mag;
                }
            }
            if (factor * largestMagnitude > nextMagnitude)
            {
                return selectedRow;
            }
            else
            {
                // -1 for underconstrained
                return -1;
            }
        }

        public double[] getXYZ(Matrix<double> m)
        {
            double[] XYZ = new double[3];
            XYZ[0] = m[0, 3]; XYZ[1] = m[1, 3]; XYZ[2] = m[2, 3];
            return XYZ;
        }
        public double[] getXYZ(MathTransform transform)
        {
            double[] XYZ = new double[3];
            XYZ[0] = transform.ArrayData[9]; XYZ[1] = transform.ArrayData[10]; XYZ[2] = transform.ArrayData[11];
            return XYZ;
        }
        public double[] getRPY(Matrix<double> m)
        {
            double roll, pitch, yaw;
            if (Math.Abs(m[2,0]) >= 1.0)
            {
                yaw = 0;
                //double delta = Math.Acos(m[2, 0]);
                pitch = -Math.Asin(m[2, 0]);
                roll = Math.Acos(m[0, 2]);
                //if (m[0,2] == 1)
                //{
                //    pitch = Math.PI / 2.0;
                //    roll = delta;
                //}
                //else
                //{
                //    pitch = -Math.PI / 2.0;
                //    roll = delta;
                //}
            }
            else 
            {
                pitch = -Math.Asin(m[2,0]);
                roll =  Math.Atan2(m[2,1] , m[2,2]);
                yaw =   Math.Atan2(m[1,0] , m[0,0]);
            
            }





            //double roll, pitch, yaw;
            //roll = Math.Atan2(m[2, 1], m[2, 2]);
            //pitch = Math.Atan2(-m[2,0], Math.Sqrt(m[2,1]*m[2,1] + m[2,2]*m[2,2]));
            //yaw = Math.Atan2(m[1, 0], m[0, 0]);

            //if (m[1, 0] > 0.99 || m[1,0] < -0.99)
            //{
            //    roll = -Math.Atan2(m[2, 1], m[2, 2]);
            //    yaw = -Math.Atan2(m[1, 0], m[0, 0]);
            //}
            return new double[] {roll, pitch, yaw};
        }
        public double[] getRPY(MathTransform transform)
        {
            Matrix m = getRotationMatrix(transform);
            return getRPY(m);
        }
        public Matrix<double> getRotation(double[] RPY)
        {
            Matrix<double> R1 = DenseMatrix.Identity(4);
            Matrix<double> R2 = DenseMatrix.Identity(4);
            Matrix<double> R3 = DenseMatrix.Identity(4);

            R1[1, 1] = Math.Cos(RPY[0]); R1[1, 2] = -Math.Sin(RPY[0]); R1[2, 1] = Math.Sin(RPY[0]); R1[2, 2] = Math.Cos(RPY[2]);
            R2[0, 0] = Math.Cos(RPY[1]); R2[0, 2] = Math.Sin(RPY[1]); R2[2, 0] = -Math.Sin(RPY[1]); R2[2, 2] = Math.Cos(RPY[1]);
            R3[0, 0] = Math.Cos(RPY[2]); R3[0, 1] = -Math.Sin(RPY[2]); R3[1, 0] = Math.Sin(RPY[2]); R3[1, 1] = Math.Cos(RPY[2]);

            return R1 * R2 * R3;
        }
        public Matrix<double> getTranslation(double[] XYZ)
        {
            Matrix<double> m = DenseMatrix.Identity(4);
            m[0, 3] = XYZ[0]; m[1, 3] = XYZ[1]; m[2, 3] = XYZ[2];
            return m;
        }
        public Matrix<double> getTransformation(double[] XYZ, double[] RPY)
        {
            Matrix<double> translation = getTranslation(XYZ);
            Matrix<double> rotation = getRotation(RPY);
            return translation * rotation;
        }
        public Matrix getRotationMatrix(MathTransform transform)
        {
            var rot = new DenseMatrix(3);
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    rot.At(i, j, transform.ArrayData[i + 3 * j]);
            return rot;
        }

        public Matrix<double> getTransformation(Vector<double> translationVector, Vector<double> rotationVector, double angle)
        {
            Matrix<double> transform = DenseMatrix.Identity(4);

            return transform;
        }
        public Matrix<double> getTransformation(MathTransform transform)
        {
            Matrix<double> m = new DenseMatrix(4);
            //m[0, 0] = transform.ArrayData[0];
            //m[0, 1] = transform.ArrayData[1];
            //m[0, 2] = transform.ArrayData[2];
            //m[1, 0] = transform.ArrayData[3];
            //m[1, 1] = transform.ArrayData[4];
            //m[1, 2] = transform.ArrayData[5];
            //m[2, 0] = transform.ArrayData[6];
            //m[2, 1] = transform.ArrayData[7];
            //m[2, 2] = transform.ArrayData[8];

            //m[0, 3] = transform.ArrayData[9];
            //m[1, 3] = transform.ArrayData[10];
            //m[2, 3] = transform.ArrayData[11];
            //m[3, 3] = transform.ArrayData[12];

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

        public double[] pnorm(double[] array, double power)
        {
            double magnitude = 0;
            for (int i = 0; i < array.Length; i++)
            {
                magnitude += Math.Pow(array[i], power);
            }
            if (magnitude != 0)
            {
                for (int i = 0; i < array.Length; i++)
                {
                    array[i] /= magnitude;
                }
            }
            return array;
        }

            
    }
}