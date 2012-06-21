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

namespace SW2URDF
{
    public class MatrixOPS
    {

    }

    public class constraints
    {
        public Vector X
        { get; set; }
        public Vector Y
        { get; set; }
        public Vector Z
        { get; set; }
        public Vector Roll
        { get; set; }
        public Vector Pitch
        { get; set; }
        public Vector Yaw
        { get; set; }
        public int constrainedDOFs
        { get; set; }

        public const Vector axis1 = (DenseVector)new double[3] { 1, 0, 0 };
        public const Vector axis2 = (DenseVector)new double[3] { 0, 1, 0 };
        public const Vector axis3 = (DenseVector)new double[3] { 0, 0, 1 };

        [Flags]
        public enum ConstrainedDOFs
        {
            X = 1,
            Y = 2,
            Z = 4,
            Roll = 8,
            Pitch = 16,
            Yaw = 32,
        }  
    }

    public class relation
    {
        private Matrix Jacobian;
        private int firstFreeRow;
        private constraints parentRelationConstraints;
        private constraints childRelationConstraints;
        private constraints jointConstraints;

        public relation()
        {
            Jacobian = new DenseMatrix(12);
            firstFreeRow = 0;
        }

        public void constrainRelationFromMate(Mate2 mate, IComponent2 comp1, IComponent2 comp2)
        {
            switch (mate.Type)
            {
                case (int)swMateType_e.swMateCOINCIDENT:
                    MateEntity2 entity1 = mate.MateEntity(0);
                    MateEntity2 entity2 = mate.MateEntity(1);
                    //constrainRelation(entity1, entity2);

                    for (int i = 0; i < mate.GetMateEntityCount(); i++)
                    {
                        MateEntity2 entity = mate.MateEntity(i);

                    }
                    break;
                case (int)swMateType_e.swMateCONCENTRIC:
                    break;
                case (int)swMateType_e.swMatePERPENDICULAR:
                    break;
                case (int)swMateType_e.swMatePARALLEL:
                    break;
                case (int)swMateType_e.swMateTANGENT:
                    break;
                case (int)swMateType_e.swMateDISTANCE:
                    break;
                case (int)swMateType_e.swMateANGLE:
                    break;
                case (int)swMateType_e.swMateUNKNOWN:
                    break;
                case (int)swMateType_e.swMateSYMMETRIC:
                    break;
                case (int)swMateType_e.swMateCAMFOLLOWER:
                    break;
                case (int)swMateType_e.swMateGEAR:
                    break;
                case (int)swMateType_e.swMateWIDTH:
                    break;
                case (int)swMateType_e.swMateLOCKTOSKETCH:
                    break;
                case (int)swMateType_e.swMateRACKPINION:
                    break;
                case (int)swMateType_e.swMateMAXMATES:
                    break;
                case (int)swMateType_e.swMatePATH:
                    break;
                case (int)swMateType_e.swMateLOCK:
                    break;
                case (int)swMateType_e.swMateSCREW:
                    break;
                case (int)swMateType_e.swMateLINEARCOUPLER:
                    break;
                case (int)swMateType_e.swMateUNIVERSALJOINT:
                    break;
                case (int)swMateType_e.swMateCOORDINATE:
                    break;
                case (int)swMateType_e.swMateSLOT:
                    break;
                case (int)swMateType_e.swMateHINGE:
                    break;
                case (int)swMateType_e.swMateSLIDER:
                    break;

            }
        }

        public void addConstraintsToJacobian(constraints c1, constraints c2)
        {
            int constrainedDOFs = jointConstraints.constrainedDOFs | ( c1.constrainedDOFs & c2.constrainedDOFs);

            if ((constrainedDOFs & (int)constraints.ConstrainedDOFs.X) != 0)
            {
                addConstraintVectorToJacobian(vectorCat(c1.X, c2.X));
            }
            if ((constrainedDOFs & (int)constraints.ConstrainedDOFs.Y) != 0)
            {
            }
            if ((constrainedDOFs & (int)constraints.ConstrainedDOFs.Z) != 0)
            {
            }
            if ((constrainedDOFs & (int)constraints.ConstrainedDOFs.Roll) != 0)
            {
            }
            if ((constrainedDOFs & (int)constraints.ConstrainedDOFs.Pitch) != 0)
            {
            }
            if ((constrainedDOFs & (int)constraints.ConstrainedDOFs.Yaw) != 0)
            {
            }

        }

        public Vector vectorCat(Vector v1, Vector v2)
        {
            Vector vec = new DenseVector(v1.Count + v2.Count);
            v1.CopyTo(vec);
            v2.CopyTo(vec, 0, v1.Count, v2.Count);
            return vec;
        }
        public Matrix rref(Matrix m)
        {
            return m;
        }

        public Matrix nullSpace(Matrix m)
        {
            return m;
        }

        public void addConstraintVectorToJacobian(Vector v)
        {
            if (isLinearlyIndependent(Jacobian, v))
            {
                Jacobian.SetRow(firstFreeRow, v);
                firstFreeRow++;
            }
        }

        public bool isLinearlyIndependent(Matrix m, Vector v)
        {
            return true;
        }
        public constraints setConstraints(MateEntity2 entity)
        {
            constraints entityConstraints = new constraints();
            switch (entity.ReferenceType2)
            {
                case (int)swMateEntity2ReferenceType_e.swMateEntity2ReferenceType_Point:
                    entityConstraints.constrainedDOFs = ((int)constraints.ConstrainedDOFs.X | (int)constraints.ConstrainedDOFs.Y | (int)constraints.ConstrainedDOFs.Z);
                    entityConstraints.X = (DenseVector)(getRotationMatrix(entity.ReferenceComponent.Transform2) * constraints.axis1);
                    entityConstraints.Y = (DenseVector)(getRotationMatrix(entity.ReferenceComponent.Transform2) * constraints.axis2);
                    entityConstraints.Z = (DenseVector)(getRotationMatrix(entity.ReferenceComponent.Transform2) * constraints.axis3);
                    break;
                case (int)swMateEntity2ReferenceType_e.swMateEntity2ReferenceType_Line:
                    entityConstraints.constrainedDOFs = ((int)constraints.ConstrainedDOFs.X | (int)constraints.ConstrainedDOFs.Y | (int)constraints.ConstrainedDOFs.Roll | (int)constraints.ConstrainedDOFs.Pitch);
                    entityConstraints.X = (DenseVector)(getRotationMatrix(entity.ReferenceComponent.Transform2) * constraints.axis1);
                    entityConstraints.Y = (DenseVector)(getRotationMatrix(entity.ReferenceComponent.Transform2) * constraints.axis2);
                    entityConstraints.Roll = (DenseVector)(getRotationMatrix(entity.ReferenceComponent.Transform2) * constraints.axis1);
                    entityConstraints.Pitch = (DenseVector)(getRotationMatrix(entity.ReferenceComponent.Transform2) * constraints.axis2);
                    break;
                case (int)swMateEntity2ReferenceType_e.swMateEntity2ReferenceType_Circle:
                    entityConstraints.constrainedDOFs = ((int)constraints.ConstrainedDOFs.X | (int)constraints.ConstrainedDOFs.Y | (int)constraints.ConstrainedDOFs.Z | (int)constraints.ConstrainedDOFs.Roll | (int)constraints.ConstrainedDOFs.Pitch);
                    entityConstraints.X = (DenseVector)(getRotationMatrix(entity.ReferenceComponent.Transform2) * constraints.axis1);
                    entityConstraints.Y = (DenseVector)(getRotationMatrix(entity.ReferenceComponent.Transform2) * constraints.axis2);
                    entityConstraints.Z = (DenseVector)(getRotationMatrix(entity.ReferenceComponent.Transform2) * constraints.axis3);
                    entityConstraints.Roll = (DenseVector)(getRotationMatrix(entity.ReferenceComponent.Transform2) * constraints.axis1);
                    entityConstraints.Pitch = (DenseVector)(getRotationMatrix(entity.ReferenceComponent.Transform2) * constraints.axis2);
                    break;
                case (int)swMateEntity2ReferenceType_e.swMateEntity2ReferenceType_Plane:
                    entityConstraints.constrainedDOFs = ((int)constraints.ConstrainedDOFs.X | (int)constraints.ConstrainedDOFs.Roll | (int)constraints.ConstrainedDOFs.Pitch);
                    entityConstraints.X = (DenseVector)(getRotationMatrix(entity.ReferenceComponent.Transform2) * constraints.axis1);
                    entityConstraints.Roll = (DenseVector)(getRotationMatrix(entity.ReferenceComponent.Transform2) * constraints.axis2);
                    entityConstraints.Pitch = (DenseVector)(getRotationMatrix(entity.ReferenceComponent.Transform2) * constraints.axis3);
                    break;
                case (int)swMateEntity2ReferenceType_e.swMateEntity2ReferenceType_Cylinder:
                    entityConstraints.constrainedDOFs = ((int)constraints.ConstrainedDOFs.X | (int)constraints.ConstrainedDOFs.Y | (int)constraints.ConstrainedDOFs.Roll | (int)constraints.ConstrainedDOFs.Pitch);
                    entityConstraints.X = (DenseVector)(getRotationMatrix(entity.ReferenceComponent.Transform2) * constraints.axis1);
                    entityConstraints.Y = (DenseVector)(getRotationMatrix(entity.ReferenceComponent.Transform2) * constraints.axis2);
                    entityConstraints.Roll = (DenseVector)(getRotationMatrix(entity.ReferenceComponent.Transform2) * constraints.axis1);
                    entityConstraints.Pitch = (DenseVector)(getRotationMatrix(entity.ReferenceComponent.Transform2) * constraints.axis2);
                    break;
                case (int)swMateEntity2ReferenceType_e.swMateEntity2ReferenceType_Sphere:
                    break;
                case (int)swMateEntity2ReferenceType_e.swMateEntity2ReferenceType_Set:
                    break;
                case (int)swMateEntity2ReferenceType_e.swMateEntity2ReferenceType_Cone:
                    break;
                case (int)swMateEntity2ReferenceType_e.swMateEntity2ReferenceType_SweptSurface:
                    break;
                case (int)swMateEntity2ReferenceType_e.swMateEntity2ReferenceType_MultipleSurface:
                    break;
                case (int)swMateEntity2ReferenceType_e.swMateEntity2ReferenceType_GenSurface:
                    break;
                case (int)swMateEntity2ReferenceType_e.swMateEntity2ReferenceType_GeneralCurve:
                    break;
                case (int)swMateEntity2ReferenceType_e.swMateEntity2ReferenceType_UNKNOWN:
                    break;
            }
            return entityConstraints;
        }

        public Matrix getRotationMatrix(MathTransform transform)
        {
            var rot = new DenseMatrix(3);
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    rot.At(i, j, transform.ArrayData[i + 3*j]);
            return rot;
        }
                
        public void constrainRelationFromCoincidentMate(MateEntity2 entity1, MateEntity2 entity2)
        {
            switch (entity1.ReferenceType2)
            {
                case (int)swMateEntity2ReferenceType_e.swMateEntity2ReferenceType_Point:
                    break;
                case (int)swMateEntity2ReferenceType_e.swMateEntity2ReferenceType_Line:
                    break;
                case (int)swMateEntity2ReferenceType_e.swMateEntity2ReferenceType_Circle:
                    break;
                case (int)swMateEntity2ReferenceType_e.swMateEntity2ReferenceType_Plane:
                    break;
                case (int)swMateEntity2ReferenceType_e.swMateEntity2ReferenceType_Cylinder:
                    break;
                case (int)swMateEntity2ReferenceType_e.swMateEntity2ReferenceType_Sphere:
                    break;
                case (int)swMateEntity2ReferenceType_e.swMateEntity2ReferenceType_Set:
                    break;
                case (int)swMateEntity2ReferenceType_e.swMateEntity2ReferenceType_Cone:
                    break;
                case (int)swMateEntity2ReferenceType_e.swMateEntity2ReferenceType_SweptSurface:
                    break;
                case (int)swMateEntity2ReferenceType_e.swMateEntity2ReferenceType_MultipleSurface:
                    break;
                case (int)swMateEntity2ReferenceType_e.swMateEntity2ReferenceType_GenSurface:
                    break;
                case (int)swMateEntity2ReferenceType_e.swMateEntity2ReferenceType_GeneralCurve:
                    break;
                case (int)swMateEntity2ReferenceType_e.swMateEntity2ReferenceType_UNKNOWN:
                    break;
            }
        }


    }

}
