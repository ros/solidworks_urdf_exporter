using MathNet.Numerics.LinearAlgebra.Double;
using MathNet.Numerics.LinearAlgebra.Generic;
using SolidWorks.Interop.sldworks;
using SW2URDF.Utilities;
using System;
using System.Collections.Generic;
using Xunit;
namespace SW2URDF.Test
{
    public class TestMathOps : SW2URDFTest
    {
        public TestMathOps(SWTestFixture fixture) : base(fixture)
        {
        }

        [Theory]
        [InlineData(0, 0, 1, 1)]
        [InlineData(1, 0, -1, 1)]
        [InlineData(5, 0, -1, 5)]
        [InlineData(2, 3, 1, 3)]
        [InlineData(4, -3, 1, 4)]
        public void TestMax3Int(int a, int b, int c, int expected)
        {
            Assert.Equal(expected, MathOps.Max(a, b, c));
        }

        [Theory]
        [InlineData(0.0, 0.0, 1.0, 1.0)]
        [InlineData(1.0, 0.0, -1.0, 1.0)]
        [InlineData(5.0, 0.0, -1.0, 5.0)]
        [InlineData(2.0, 3.0, 1.0, 3.0)]
        [InlineData(4, -3.0, 1.0, 4.0)]
        public void TestMax3Double(double a, double b, double c, double expected)
        {
            Assert.Equal(expected, MathOps.Max(a, b, c));
        }

        [Theory]
        [InlineData(new int[] { 1 }, 1)]
        [InlineData(new int[] { 1, -1 }, 1)]
        [InlineData(new int[] { 5, 0, -1 }, 5)]
        [InlineData(new int[] { 2, 3, 1, -1 }, 3)]
        [InlineData(new int[] { 4, -3, 1, 0, -5 }, 4)]
        public void TestMaxArrayInt(int[] array, int expected)
        {
            Assert.Equal(expected, MathOps.Max(array));
        }

        [Theory]
        [InlineData(new double[] { 1.0 }, 1.0)]
        [InlineData(new double[] { 1.0, -1.0 }, 1.0)]
        [InlineData(new double[] { 5.0, 0.0, -1.0 }, 5.0)]
        [InlineData(new double[] { 2.0, 3.0, 1.0, -1.0 }, 3.0)]
        [InlineData(new double[] { 4.0, -3.0, 1.0, 0.0, -5.0 }, 4.0)]
        public void TestMaxArrayDouble(double[] array, int expected)
        {
            Assert.Equal(expected, MathOps.Max(array));
        }




        [Theory]
        [InlineData(0, 0, 1, 0)]
        [InlineData(1, 0, -1, -1)]
        [InlineData(5, 0, -1, -1)]
        [InlineData(2, 3, 1, 1)]
        [InlineData(4, -3, 1, -3)]
        public void TestMin3Int(int a, int b, int c, int expected)
        {
            Assert.Equal(expected, MathOps.Min(a, b, c));
        }

        [Theory]
        [InlineData(0.0, 0.0, 1.0, 0.0)]
        [InlineData(1.0, 0.0, -1.0, -1.0)]
        [InlineData(5.0, 0.0, -1.0, -1.0)]
        [InlineData(2.0, 3.0, 1.0, 1.0)]
        [InlineData(4, -3.0, 1.0, -3.0)]
        public void TestMin3Double(double a, double b, double c, double expected)
        {
            Assert.Equal(expected, MathOps.Min(a, b, c));
        }

        [Theory]
        [InlineData(new int[] { 1 }, 1)]
        [InlineData(new int[] { 1, -1 }, -1)]
        [InlineData(new int[] { 5, 0, -1 }, -1)]
        [InlineData(new int[] { 2, 3, 1, -1 }, -1)]
        [InlineData(new int[] { 4, -3, 1, 0, -5 }, -5)]
        public void TestMinArrayInt(int[] array, int expected)
        {
            Assert.Equal(expected, MathOps.Min(array));
        }

        [Theory]
        [InlineData(new double[] { 1.0 }, 1.0)]
        [InlineData(new double[] { 1.0, -1.0 }, -1.0)]
        [InlineData(new double[] { 5.0, 0.0, -1.0 }, -1.0)]
        [InlineData(new double[] { 2.0, 3.0, 1.0, -1.0 }, -1.0)]
        [InlineData(new double[] { 4.0, -3.0, 1.0, 0.0, -5.0 }, -5.0)]
        public void TestMinArrayDouble(double[] array, int expected)
        {
            Assert.Equal(expected, MathOps.Min(array));
        }

        [Theory]
        [InlineData(0, 0, 1, 0)]
        [InlineData(1, -1, 0, 0)]
        [InlineData(5, 0, 1, 1)]
        [InlineData(2, 1, 3, 2)]
        [InlineData(-4, -3, 1, -3)]
        public void TestEnvelopeInt(int value, int min, int max, int expected)
        {
            Assert.Equal(expected, MathOps.Envelope(value, min, max));
        }

        [Theory]
        [InlineData(0.0, 0.0, 1.0, 0.0)]
        [InlineData(1.0, -1.0, 0.0, 0.0)]
        [InlineData(5.0, 0.0, 1.0, 1.0)]
        [InlineData(2.0, 1.0, 3.0, 2.0)]
        [InlineData(-4.0, -3.0, 1.0, -3.0)]
        public void TestEnvelopeDouble(int value, int min, int max, int expected)
        {
            Assert.Equal(expected, MathOps.Envelope(value, min, max));
        }

        [Theory]
        [InlineData(new double[] { 0.0, 0.0 }, new double[] { 1.0, 0.0 }, new double[] { 0.0, 0.0 }, new double[] { 0.0, 0.0 })]
        [InlineData(new double[] { 0.0, 0.0 }, new double[] { 0.0, 1.0 }, new double[] { 0.0, 0.0 }, new double[] { 0.0, 0.0 })]
        [InlineData(new double[] { 1.0, 0.0 }, new double[] { 1.0, 0.0 }, new double[] { 0.0, 0.0 }, new double[] { 1.0, 0.0 })]
        [InlineData(new double[] { 1.0, 0.0 }, new double[] { 0.0, 1.0 }, new double[] { 0.0, 0.0 }, new double[] { 0.0, 0.0 })]
        [InlineData(new double[] { 1.0, 1.0 }, new double[] { 1.0, 0.0 }, new double[] { 0.0, 0.0 }, new double[] { 1.0, 0.0 })]
        [InlineData(new double[] { 1.0, 1.0 }, new double[] { 0.0, 1.0 }, new double[] { 0.0, 0.0 }, new double[] { 0.0, 1.0 })]
        public void TestClosestPointOnLineToPoint(double[] point, double[] line, double[] pointOnLine, double[] expected)
        {
            double[] result = MathOps.ClosestPointOnLineToPoint(point, line, pointOnLine);
            for (int i = 0; i < expected.Length; i++)
            {
                Assert.Equal(expected[i], result[i], 10);
            }
        }

        [Theory]
        [InlineData(0.0, 1.0, 0.0, 1.0, 0.0, 1.0, new double[] { 1.0, 1.0, 1.0 }, new double[] { 0.0, 0.0, 0.0 }, new double[] { 0.0, 0.0, 0.0 })]
        [InlineData(0.0, 1.0, 0.0, 1.0, 0.0, 1.0, new double[] { 1.0, 1.0, 1.0 }, new double[] { 1.0, 1.0, 1.0 }, new double[] { 1.0, 1.0, 1.0 })]
        public void TestClosestPointOnLineWithinBox(double xMin, double xMax, double yMin, double yMax, double zMin, double zMax, double[] line, double[] pointOnLine, double[] expected)
        {
            double[] result = MathOps.ClosestPointOnLineWithinBox(xMin, xMax, yMin, yMax, zMin, zMax, line, pointOnLine);
            for (int i = 0; i < expected.Length; i++)
            {
                Assert.Equal(expected[i], result[i], 10);
            }
        }

        [Theory]
        [InlineData(new double[] { 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0 }, new double[] { 12.0, 13.0, 14.0 })]
        // https://numerics.mathdotnet.com/api/MathNet.Numerics.LinearAlgebra.Double/DenseMatrix.htm#Create
        public void TestGetXYZDenseMatrix(double[] matrixData, double[] expected)
        {
            Matrix<double> m = new DenseMatrix(4, 4, matrixData);
            double[] result = MathOps.GetXYZ(m);
            for (int i = 0; i < expected.Length; i++)
            {
                Assert.Equal(expected[i], result[i], 10);
            }
        }

        [Theory]
        [InlineData(new double[] { 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0 }, new double[] { 9.0, 10.0, 11.0 })]
        // http://help.solidworks.com/2012/english/api/sldworksapi/solidworks.interop.sldworks~solidworks.interop.sldworks.imathutility~icreatetransform.html
        public void TestGetXYZMathTransform(double[] matrixData, double[] expected)
        {
            MathUtility util = SwApp.GetMathUtility();
            MathTransform transform = util.CreateTransform((object)matrixData);
            double[] result = MathOps.GetXYZ(transform);
            for (int i = 0; i < expected.Length; i++)
            {
                Assert.Equal(expected[i], result[i], 10);
            }
        }

        [Theory]
        [InlineData(new double[] {1, 0, 0 ,0, 0, 1, 0 ,0, 0, 0, 1, 0, 0, 0, 0, 1}, new double[] {0, 0, 0})]
        [InlineData(new double[] { 1, 0, 0, 0,    0, 0, 1, 0,      -1, 0, 0, 0,    0, 0, 0, 1 }, new double[] { Math.PI / 2.0, 0, 0 })]
        [InlineData(new double[] { 0, 0, -1, 0,    0, 1, 0, 0,    1, 0, 0, 0,    0, 0, 0, 1 }, new double[] { 0, Math.PI / 2.0, 0 })]
        [InlineData(new double[] { 0, 1, 0, 0,    -1, 0, 0, 0,     0, 0, 1, 0,     0, 0, 0, 1 }, new double[] { 0, 0, Math.PI / 2.0 })]
        public void TestGetRPYDenseMatrix(double[] matrixData, double[] expected)
        {
            Matrix<double> m = new DenseMatrix(4, 4, matrixData);
            double[] result = MathOps.GetRPY(m);
            for (int i = 0; i < expected.Length; i++)
            {
                Assert.Equal(expected[i], result[i], 10);
            }
        }

        [Theory]
        //Ordering (Row, Column) 
        // {(0,0), (0,1), (0,2), (1,0), (1,1), (1,2), (2,0), (2,1), (2, 2), (3,0) (3,1), (3,2), (3,3), (0,3), (1,3), (2,3)
        [InlineData(new double[] { 1, 0, 0,    0, 1, 0,    0, 0, 1,   0, 0, 0,    1, 0, 0, 0 }, new double[] { 0, 0, 0 })]
        public void TestGetRPYMathTransform(double[] matrixData, double[] expected)
        {
            MathUtility util = SwApp.GetMathUtility();
            MathTransform transform = util.CreateTransform((object)matrixData);
            double[] result = MathOps.GetRPY(transform);
            for (int i = 0; i < expected.Length; i++)
            {
                Assert.Equal(expected[i], result[i], 10);
            }
        }

        [Theory]
        [InlineData(new double[] { 0, 0, 0 },             new double[] { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 })]
        [InlineData(new double[] { Math.PI / 2.0, 0, 0 }, new double[] { 1, 0, 0, 0,  0, 0, 1, 0,  0, -1, 0, 0,  0, 0, 0, 1 })]
        [InlineData(new double[] { 0, Math.PI / 2.0, 0 }, new double[] { 0, 0, -1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1 })]
        [InlineData(new double[] { 0, 0, Math.PI / 2.0 }, new double[] { 0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 })]
        public void TestGetRotation(double[] rpy, double[] expected)
        {
            double[] result = MathOps.GetRotation(rpy).ToColumnWiseArray();
            for (int i = 0; i < expected.Length; i++)
            {
                Assert.Equal(expected[i], result[i], 10);
            }
        }

        [Theory]
        [InlineData(new double[] {0, 0, 0 }, new double[] { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 })]
        [InlineData(new double[] { 1, 2, 3 }, new double[] { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1, 2, 3, 1 })]
        public void TestGetTranslation(double[] xyz, double[] expected)
        {
            double[] result = MathOps.GetTranslation(xyz).ToColumnWiseArray();
            for (int i = 0; i < expected.Length; i++)
            {
                Assert.Equal(expected[i], result[i], 10);
            }
        }

        [Theory]
        [InlineData(new double[] { 0, 0, 0 },new double[] { 0.0, 0.0, 0.0 }, new double[] { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 })]
        [InlineData(new double[] { 1, 2, 3 }, new double[] { 0.0, 0.0, 0.0 }, new double[] { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1, 2, 3, 1 })]
        [InlineData(new double[] { 0, 0, 0 }, new double[] { Math.PI / 2.0, 0, 0 }, new double[] { 1, 0, 0, 0, 0, 0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 1 })]
        [InlineData(new double[] { 0, 0, 0 }, new double[] { 0.0, Math.PI / 2.0, 0}, new double[] { 0, 0, -1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1 })]
        [InlineData(new double[] { 0, 0, 0 }, new double[] { 0.0, 0.0,  Math.PI / 2.0}, new double[] { 0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 })]
        public void TestGetTransformationFromXYZRPY(double[] xyz, double[] rpy, double[] expected)
        {
            double[] result = MathOps.GetTransformation(xyz, rpy).ToColumnWiseArray();
            for (int i = 0; i < expected.Length; i++)
            {
                Assert.Equal(expected[i], result[i], 10);
            }
        }

        [Theory]
        // Matrix Ordering (Row, Column)
        // (0, 0), (1, 0), (2, 0), (3, 0), (0, 1), (1, 1),(2, 1),(3, 1), (0, 2), (1, 2), (2, 2), (3, 2),  (0, 3), (1, 3), (2, 3), (3, 3),
        //Math Transform Ordering (Row, Column) 
        // {(0,0), (0,1), (0,2), (1,0), (1,1), (1,2), (2,0), (2,1), (2, 2), (3,0) (3,1), (3,2), (3,3), (0,3), (1,3), (2,3)
        [InlineData(new double[] { 1, 0, 0,   0, 1, 0,   0, 0, 1,   0, 0, 0, 1,   0, 0, 0 },  new double[] { 1, 0, 0, 0, 1, 0, 0, 0, 1})]
        [InlineData(new double[] { 1, 0, 0,   0, 1, 0,   0, 0, 1,   0, 0, 0, 1,   2, 3, 4 },  new double[] { 1, 0, 0, 0, 1, 0, 0, 0, 1 })]
        [InlineData(new double[] { 1, 0, 0,   0, 0, -1,  0, 1, 0,   0, 0, 0, 1,   0, 0, 0},   new double[] { 1, 0, 0, 0, 0, -1, 0, 1, 0})]
        [InlineData(new double[] { 0, 0, -1,  0, 1, 0,   1, 0, 0,   0, 0, 0, 1,   0, 0, 0 },  new double[] { 0, 0, -1, 0, 1, 0, 1, 0, 0})]
        [InlineData(new double[] { 0, 1, 0,   -1, 0, 0,  0, 0, 1,   0, 0, 0, 1,   0, 0, 0 },  new double[] { 0, 1, 0, -1, 0, 0, 0, 0, 1})]

        public void TestGetRotationMatrixFromMathTransform(double[] mathTransformData, double[] expected)
        {
            MathUtility util = SwApp.GetMathUtility();
            MathTransform transform = util.CreateTransform((object)mathTransformData);
            
            double[] result = MathOps.GetRotationMatrix(transform).ToColumnWiseArray();
            for (int i = 0; i < expected.Length; i++)
            {
                Assert.Equal(expected[i], result[i]);
            }
        }

        [Theory]
        [InlineData(new double[] { 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0 },  new double[] { 1, 0, 0, 0,   0, 1, 0, 0,  0, 0, 1, 0,  0, 0, 0, 1 })]
        [InlineData(new double[] { 1, 0, 0, 0, 1, 0, 0, 0, 1, 2, 3, 4, 1, 0, 0, 0 },  new double[] { 1, 0, 0, 0,   0, 1, 0, 0,  0, 0, 1, 0,  2, 3, 4, 1 })]
        [InlineData(new double[] { 1, 0, 0, 0, 0, -1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0 }, new double[] { 1, 0, 0, 0,   0, 0, -1, 0,  0, 1, 0, 0, 0, 0, 0, 1 })]
        [InlineData(new double[] { 0, 0, -1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0 }, new double[] { 0, 0, -1, 0,  0, 1, 0, 0,  1, 0, 0, 0,  0, 0, 0, 1 })]
        [InlineData(new double[] { 0, 1, 0, -1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0 }, new double[] { 0, 1, 0, 0,  -1, 0, 0, 0,  0, 0, 1, 0,  0, 0, 0, 1 })]
        public void TestGetTransformationFromMathTransform(double[] matrixData, double[] expected)
        {
            MathUtility util = SwApp.GetMathUtility();
            MathTransform transform = util.CreateTransform((object)matrixData);
            double[] result = MathOps.GetTransformation(transform).ToColumnWiseArray();
            for (int i = 0; i < expected.Length; i++)
            {
                Assert.Equal(expected[i], result[i]);
            }
        }

        [Theory]
        [InlineData(new double[] {0, 0, 0}, 1.0, new double[] {0, 0, 0})]
        [InlineData(new double[] { 1, 0, 0 }, 1.0, new double[] { 1, 0, 0 })]
        [InlineData(new double[] { 0, 0, 0 }, 2.0, new double[] { 0, 0, 0 })]
        [InlineData(new double[] { 1, 1, 1 }, 1.0, new double[] { 0.33333333333, 0.33333333333, 0.33333333333 })]
        [InlineData(new double[] { 1, 1, 1 }, 2.0, new double[] { 0.57735026919, 0.57735026919, 0.57735026919 })]
        [InlineData(new double[] { 2, 2, 2 }, 2.0, new double[] { 0.57735026919, 0.57735026919, 0.57735026919 })]
        public void TestPNorm(double[] array, double power, double[] expected)
        {
            double[] result = MathOps.PNorm(array, power);
            for (int i = 0; i < expected.Length; i++)
            {
                Assert.Equal(expected[i], result[i], 10);
            }
        }

        [Theory]
        [InlineData(new double[] {1.0, 0 }, new double[] {1.0, 0 }, 0)]
        [InlineData(new double[] { 1.0, 0 }, new double[] { 0, 1.0 }, 2)]
        [InlineData(new double[] { 0.0, 0 }, new double[] { 1.0, 1.0 }, 2)]
        [InlineData(new double[] { 1.0, 2.0, 3.0 }, new double[] {3.0, 2.0, 1.0}, 8)]

        public void TestDistance2(double[] array1, double[] array2, double expected)
        {
            Assert.Equal(expected, MathOps.Distance2(array1, array2));
        }

        [Theory]
        [InlineData(new double[]{0.1, 1, 10.0}, 0.5, new double[] {0, 1, 10 })]
        [InlineData(new double[]{0.1, 1, 10.0}, 0.1, new double[] { 0.1, 1, 10 })]
        [InlineData(new double[]{0.1, 0.01, 0.001}, 0.5, new double[] { 0.0, 0.0, 0.0 })]
        [InlineData(new double[]{0.1, 0.01, 0.001}, 0.1, new double[] { 0.1, 0.0, 0.0 })]
        public void TestThreshold(double[] array, double minValue, double[] expected)
        {
            double[] result = MathOps.Threshold(array, minValue);
            for (int i = 0; i < expected.Length; i++)
            {
                Assert.Equal(expected[i], result[i]);
            }
        }
    }
}
