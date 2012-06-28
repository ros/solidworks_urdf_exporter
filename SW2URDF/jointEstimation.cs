using System;
using System.IO;
using System.Runtime.InteropServices;
using System.Collections;
using System.Reflection;

using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.swpublished;
using SolidWorks.Interop.swconst;
using SolidWorksTools;
using SolidWorksTools.File;
using System.Collections.Generic;
using System.Diagnostics;
using System.Windows.Forms;
using MathNet.Numerics.LinearAlgebra.Generic;
using MathNet.Numerics.LinearAlgebra.Double;
using MatrixOPS;

namespace SW2URDF
{
    class jointEstimation
    {
        ISldWorks iSwApp = null;
        MathUtility swMathUtility = null;
        public Vector X
        { get; set; }
        public Vector Y
        { get; set; }
        public Vector Z
        { get; set; }
        public Matrix transform
        { get; set; }

        private Matrix axes = DenseMatrix.Identity(3);
        private Vector axis1 = new DenseVector(new double[3] { 1, 0, 0 });
        private Vector axis2 = new DenseVector(new double[3] { 0, 1, 0 });
        private Vector axis3 = new DenseVector(new double[3] { 0, 0, 1 });
        private ops OPS;

        public jointEstimation(ISldWorks iSldWorksApp)
        {
            iSwApp = iSldWorksApp;
            swMathUtility = iSwApp.GetMathUtility();
            OPS = new ops();
        }

        public joint estimateJointFromComponents(AssemblyDoc assy, IComponent2 comp1, IComponent2 comp2, bool checkBothDirections)
        {
            Matrix rotationAxes = new DenseMatrix(3, 3);
            Matrix translationAxes = new DenseMatrix(3, 3);
            for (int i = 0; i < 3; i++)
            {
                MathTransform comp1TransformBefore = comp1.Transform2;
                double[] data1 = comp1TransformBefore.ArrayData;
                MathTransform comp2TransformBefore = comp2.Transform2;
                double[] data2 = comp2TransformBefore.ArrayData;

                MathTransform comp2TransformTry = comp2TransformBefore;
                comp2TransformTry.ArrayData[9 + i] += 1;
                comp2.SetTransformAndSolve2(comp2TransformTry);

                // If dragging the second component moves the first one, then the link should be reversed
                if (checkBothDirections)
                {
                    if (!equals(comp1.Transform2,comp1TransformBefore))
                    {
                        comp1.SetTransformAndSolve2(comp1TransformBefore);
                        comp2.SetTransformAndSolve2(comp2TransformBefore);
                        return estimateJointFromComponents(assy, comp2, comp1, false);
                    }
                }
                double[] data3 = comp2.Transform2.ArrayData;

                MathTransform intermediateTransform = comp2.Transform2.Multiply(comp2TransformBefore.Inverse());

                Matrix rot = getRotationMatrix(intermediateTransform);
                Matrix nullSpaceMatrix = OPS.nullSpace(rot);
                Vector nullSpace = (DenseVector)nullSpaceMatrix.Column(2);
                translationAxes.SetRow(i, new double[] { intermediateTransform.ArrayData[9], intermediateTransform.ArrayData[10], intermediateTransform.ArrayData[11] });

                // Reset component2
                comp2.SetTransformAndSolve2(comp2TransformBefore);
            }
            for (int i = 0; i < 3; i++)
            {
                MathTransform comp1TransformBefore = comp1.Transform2;
                double[] data1 = comp1TransformBefore.ArrayData;
                MathTransform comp2TransformBefore = comp2.Transform2;
                double[] data2 = comp2TransformBefore.ArrayData;

                MathVector rotVector = swMathUtility.CreateVector(axes.Row(i).ToArray());
                MathPoint position = swMathUtility.CreatePoint(new double[]{0,0,0});
                MathTransform comp2TransformTry = swMathUtility.CreateTransformRotateAxis(position, rotVector, 10);
                comp2.SetTransformAndSolve2(comp2TransformTry);

                // If moving the second component moves the first one, then the link should be reversed
                if (checkBothDirections)
                {
                    if (!equals(comp1.Transform2,comp1TransformBefore))
                    {
                        comp1.SetTransformAndSolve2(comp1TransformBefore);
                        comp2.SetTransformAndSolve2(comp2TransformBefore);
                        return estimateJointFromComponents(assy, comp2, comp1, false);
                    }
                }

                double[] data3 = comp2.Transform2.ArrayData;

                MathTransform intermediateTransform = comp2.Transform2.Multiply(comp2TransformBefore.Inverse());

                Matrix rot = getRotationMatrix(intermediateTransform);
                Matrix nullSpaceMatrix = OPS.nullSpace(rot);
                rotationAxes.SetRow(i, nullSpaceMatrix.Column(2));
                // Reset component2
                comp2.SetTransformAndSolve2(comp2TransformBefore);
            }

            int rotationRank = rotationAxes.Rank();
            int translationRank = translationAxes.Rank();
            joint Joint = new joint();
            if (rotationRank + translationRank != 1)
            {
                Joint.type = "Fixed";
            }
            else if (rotationRank == 1)
            {
                Joint.type = "Revolute";

                rotationAxes = OPS.rref(rotationAxes);
                Joint.Axis.XYZ = rotationAxes.Row(0).ToArray();
            }
            else
            {
                Joint.type = "Prismatic";

                translationAxes = OPS.rref(translationAxes);
                Joint.Axis.XYZ = translationAxes.Row(0).ToArray();
            }
            return Joint;
        }

        public void dragComponentInDirection(AssemblyDoc assy, IComponent2 comp, Vector direction)
        {
            DragOperator drag = assy.GetDragOperator();

            // 0 for moving the minimal amount of components necessary
            // 2 to solve by relaxation
            drag.DragMode = 0;
            //drag.GraphicsRedrawEnabled = true;
            //drag.SmartMating = true;
            //drag.IsDragSpecific = true;
            //drag.

            // 0 for a translation move (though the ultimate move may not be translation)
            drag.TransformType = 0;

            drag.AddComponent(comp, false);

            double[] matValues = new double[16];
            matValues[9] = direction[0]; matValues[10] = direction[1]; matValues[11] = direction[2];

            MathTransform transform = swMathUtility.CreateTransform(matValues);
            
            drag.BeginDrag();
            //drag.DragAsUI(transform);
            drag.Drag(transform);
            drag.EndDrag();

        }
        public Matrix getRotationMatrix(MathTransform transform)
        {
            var rot = new DenseMatrix(3);
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    rot.At(i, j, transform.ArrayData[i + 3 * j]);
            return rot;
        }

        public bool equals(MathTransform t1, MathTransform t2)
        {
            for (int i = 0; i < 16; i++)
                if (t1.ArrayData[i] != t2.ArrayData[i])
                    return false;
            return true;
        }
    }
}