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
            //A Collection of axes that the object may be rotated around
            Matrix<double> rotationAxes = new DenseMatrix(3, 3);
            double[] rotationConfidences = new double[3];
            // A collection of axes that the object may be translated on
            Matrix<double> translationAxes = new DenseMatrix(3, 3);

            for (int i = 0; i < 3; i++)
            {
                // Save the original transform
                MathTransform parentTransformBefore = parent.Transform2;

                // Save the original transform
                MathTransform childTransformBefore = child.Transform2;

                //Temporary
                double[] data2 = childTransformBefore.ArrayData;
                double[] copiedData = new double[16];
                Array.Copy(data2, copiedData, 16);
                double[] newData = new double[16];
                Array.Copy(data2, newData, 16);
                newData[9 + i] += 0.1;
                childTransformBefore.ArrayData = newData;

                // Perform the transform operation
                parent.Select(false);
                assy.FixComponent();
                dragComponentInDirection((AssemblyDoc)iSwApp.ActiveDoc, child, axes.Row(i) * 1e-15);
                child.SetTransformAndSolve2(childTransformBefore);

                //Temp
                double[] data3 = child.Transform2.ArrayData;

                // Compare the before with the after
                childTransformBefore.ArrayData = data2;
                MathTransform intermediateTransform = child.Transform2.Multiply(childTransformBefore.Inverse());

                // Add the translation that occured to the object
                translationAxes.SetRow(i, new double[] { intermediateTransform.ArrayData[9], intermediateTransform.ArrayData[10], intermediateTransform.ArrayData[11] });

                // Reset component2
                child.SetTransformAndSolve2(childTransformBefore);
                parent.Select(false);
                assy.UnfixComponent();
            }
            for (int i = 0; i < 3; i++)
            {
                //Save the original transform
                MathTransform parentTransformBefore = parent.Transform2;

                //Save original transform
                MathTransform childTransformBefore = child.Transform2;

                //Temporary
                double[] data2 = childTransformBefore.ArrayData;
                double[] copiedData = new double[16];
                Array.Copy(data2, copiedData, 16);

                // Get the vector and position to rotate around to create a transform
                MathVector rotVector = swMathUtility.CreateVector(axes.Row(i).ToArray());
                MathPoint position = swMathUtility.CreatePoint(new double[] { childTransformBefore.ArrayData[9], childTransformBefore.ArrayData[10], childTransformBefore.ArrayData[11] });
                MathTransform comp2TransformTry = swMathUtility.CreateTransformRotateAxis(position, rotVector, 1e-15);
                double[] trydata = comp2TransformTry.ArrayData;
                MathTransform composedTransform = comp2TransformTry.Multiply(childTransformBefore);
                double[] compdata = composedTransform.ArrayData;

                //Perform the transform
                parent.Select(false);
                assy.FixComponent();

                composedTransform = comp2TransformTry.Multiply(composedTransform);
                dragComponent((AssemblyDoc)iSwApp.ActiveDoc, child, composedTransform);


                double[] data3 = child.Transform2.ArrayData;

                //Compare the before and after
                MathTransform intermediateTransform = child.Transform2.Multiply(childTransformBefore.Inverse());

                //Find the eigen vector of the matrix whose eigen value is 1 (which should be the rotation axis)
                Matrix rot = getRotationMatrix(intermediateTransform);
                if (!OPS.equals(rot, DenseMatrix.Identity(3)))
                {
                    var eigen = rot.Evd();
                    var eigenValues = eigen.EigenValues();
                    Vector eigenVector = new DenseVector(3);
                    for (int j = 0; j < eigenValues.Count; j++)
                    {
                        if (eigenValues[j].Real - 1 < 1e-10 && eigenValues[j].Imaginary == 0)
                        {
                            eigenVector = (DenseVector)eigen.EigenVectors().Column(j);
                            rotationConfidences[i] = eigenVector.DotProduct(axes.Row(i));
                        }
                    }
                    rotationAxes.SetRow(i, eigenVector);
                }
                
                // Reset component2
                child.SetTransformAndSolve2(childTransformBefore);
                parent.Select(false);
                assy.UnfixComponent();
            }

            //Find the predominominant axis that the object is translated or rotated around
            int rotationIndex = OPS.findDominantDirection(rotationAxes, 0.1);
            int translationIndex = OPS.findDominantDirection(translationAxes, 0.1);

            //Create the joint
            joint Joint = new joint();
            if (rotationIndex < 0 && translationIndex < 0 )
            {
                Joint.type = "Fixed";
            }
            else if (rotationIndex >= 0)
            {
                Joint.type = "Revolute";

                double mag = rotationAxes.Row(rotationIndex).Norm(2);
                Vector<double> normalized = rotationAxes.Row(rotationIndex) / mag;
                Joint.Axis.XYZ = normalized.ToArray();
            }
            else
            {
                Joint.type = "Prismatic";

                double mag = translationAxes.Row(translationIndex).Norm(2);
                Vector<double> normalized = translationAxes.Row(translationIndex) / mag;
                Joint.Axis.XYZ = normalized.ToArray();
            }
            return Joint;
        }

        public void dragComponentInDirection(AssemblyDoc assy, IComponent2 comp, Vector<double> direction)
        {
            DragOperator drag = assy.GetDragOperator();

            // 0 for moving the minimal amount of components necessary
            drag.DragMode = 0;

            // 0 for a translation move (though the ultimate move may not be translation)
            drag.TransformType = 0;

            drag.AddComponent(comp, false);

            double[] matValues = new double[16];
            matValues[9] = direction[0]; matValues[10] = direction[1]; matValues[11] = direction[2];
            MathTransform transform = swMathUtility.CreateTransform(matValues);

            drag.BeginDrag();
            drag.Drag(transform);
            drag.EndDrag();
        }

        public void dragComponent(AssemblyDoc assy, IComponent2 comp, MathTransform transform)
        {
            DragOperator drag = assy.GetDragOperator();

            // 0 for moving the minimal amount of components necessary
            drag.DragMode = 0;

            // 0 for a translation move (though the ultimate move may not be translation)
            drag.TransformType = 2;

            drag.AddComponent(comp, false);
            
            drag.BeginDrag();
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