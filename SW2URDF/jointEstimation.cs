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
using MathNet.Numerics.LinearAlgebra;
using System.Numerics;
//using MathNet.Numerics.LinearAlgebra.Complex;
using MatrixOPS;

namespace SW2URDF
{
    class jointEstimation
    {
        ISldWorks iSwApp = null;
        MathUtility swMathUtility = null;
        private Matrix<double> axes = DenseMatrix.Identity(3);
        private ops OPS;

        public jointEstimation(ISldWorks iSldWorksApp)
        {
            iSwApp = iSldWorksApp;
            swMathUtility = iSwApp.GetMathUtility();
            OPS = new ops();
        }

        public joint estimateJointFromComponents(AssemblyDoc assy, IComponent2 parent, IComponent2 child, bool checkBothDirections)
        {
            Matrix rotationAxes = new DenseMatrix(3, 3);
            Matrix translationAxes = new DenseMatrix(3, 3);
            for (int i = 0; i < 3; i++)
            {
                MathTransform parentTransformBefore = parent.Transform2;
                double[] data1 = parentTransformBefore.ArrayData;
                MathTransform childTransformBefore = child.Transform2;
                double[] data2 = childTransformBefore.ArrayData;
                double[] newData = new double[16];
                Array.Copy(data2, newData, 16);
                newData[9 + i] += 0.1;
                childTransformBefore.ArrayData = newData;
                parent.Select(false);
                assy.FixComponent();
                
                
                //comp2TransformTry.ArrayData = new double[16];
                //v.
                //comp2TransformTry.SetData() = new double[16];
                //comp2TransformTry.ArrayData[9 + i] = 0.1;
                //    childTransformBefore;
                //comp2TransformTry.ArrayData[9 + i] = comp2TransformTry.ArrayData[9 + i] + 0.1;
                //double[] data4 = comp2TransformTry.ArrayData;
                //data4[9 + i] += 0.1;
                //comp2TransformTry.ArrayData = data4;
                child.SetTransformAndSolve2(childTransformBefore);

                // If dragging the second component moves the first one, then the link should be reversed
                //if (checkBothDirections)
                //{
                //    if (!equals(comp1.Transform2, comp1TransformBefore))
                //    {
                //        comp1.SetTransformAndSolve2(comp1TransformBefore);
                //        comp2.SetTransformAndSolve2(comp2TransformBefore);
                //        return estimateJointFromComponents(assy, comp2, comp1, false);
                //    }
                //}
                double[] data3 = child.Transform2.ArrayData;

                childTransformBefore.ArrayData = data2;
                // This part is not working quite right...
                MathTransform intermediateTransform = child.Transform2.Multiply(childTransformBefore.Inverse()); // It appears the multiply function left multiplies the argument
                translationAxes.SetRow(i, new double[] { intermediateTransform.ArrayData[9], intermediateTransform.ArrayData[10], intermediateTransform.ArrayData[11] });

                // Reset component2
                child.SetTransformAndSolve2(childTransformBefore);
                parent.Select(false);
                assy.UnfixComponent();
            }
            for (int i = 0; i < 3; i++)
            {
                MathTransform parentTransformBefore = parent.Transform2;
                double[] data1 = parentTransformBefore.ArrayData;
                MathTransform childTransformBefore = child.Transform2;
                double[] data2 = childTransformBefore.ArrayData;

                MathVector rotVector = swMathUtility.CreateVector(axes.Row(i).ToArray());
                MathPoint position = swMathUtility.CreatePoint(new double[] { 0, 0, 0 });
                MathTransform comp2TransformTry = swMathUtility.CreateTransformRotateAxis(position, rotVector, 180);
                parent.Select(false);
                assy.FixComponent();
                child.SetTransformAndSolve2(comp2TransformTry);

                //// If moving the second component moves the first one, then the link should be reversed
                //if (checkBothDirections)
                //{
                //    if (!equals(comp1.Transform2,comp1TransformBefore))
                //    {
                //        comp1.SetTransformAndSolve2(comp1TransformBefore);
                //        comp2.SetTransformAndSolve2(comp2TransformBefore);
                //        return estimateJointFromComponents(assy, comp2, comp1, false);
                //    }
                //}

                double[] data3 = child.Transform2.ArrayData;

                MathTransform intermediateTransform = child.Transform2.Multiply(childTransformBefore.Inverse());

                Matrix rot = getRotationMatrix(intermediateTransform);
                var eigen = rot.Evd();
                var eigenValues = eigen.EigenValues();
                Vector eigenVector = new DenseVector(3);
                for (int j = 0; j < eigenValues.Count; j++)
                {
                    if (eigenValues[j] == 1)
                    {
                        eigenVector = (DenseVector)eigen.EigenVectors().Column(j);
                    }
                }
                //Matrix nullSpaceMatrix = OPS.nullSpace(rot);
                rotationAxes.SetRow(i, eigenVector);
                // Reset component2
                child.SetTransformAndSolve2(childTransformBefore);
                parent.Select(false);
                assy.UnfixComponent();
            }

            int rotationRank = rotationAxes.Rank();

            int rotationIndex = OPS.findDominantDirection(rotationAxes, 0.1);
            int translationIndex = OPS.findDominantDirection(translationAxes, 0.1);
            //int translationRank = translationAxes.Rank();
            joint Joint = new joint();
            if (rotationIndex < 0 && translationIndex < 0 )
            {
                Joint.type = "Fixed";
            }
            else if (rotationRank >= 0)
            {
                Joint.type = "Revolute";

                Joint.Axis.XYZ = rotationAxes.Row(rotationIndex).ToArray();
            }
            else
            {
                Joint.type = "Prismatic";

                //translationAxes = OPS.rref(translationAxes);
                Joint.Axis.XYZ = translationAxes.Row(translationIndex).ToArray();
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