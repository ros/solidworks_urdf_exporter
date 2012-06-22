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
        private ops OPS;

        public relation()
        {
            Jacobian = new DenseMatrix(12);
            firstFreeRow = 0;
            OPS = new ops();
        }

        public void constrainRelationFromMate(Mate2 mate, IComponent2 comp1, IComponent2 comp2)
        {
            switch (mate.Type)
            {
                case (int)swMateType_e.swMateCOINCIDENT:
                    constraints entityConstraints1 = setConstraints(mate.MateEntity(0));
                    constraints entityConstraints2 = setConstraints(mate.MateEntity(1));
                    constraints mateConstraints = setConstraints(mate);
                    addConstraintsToJacobian(entityConstraints1, entityConstraints2, mateConstraints);
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

        public void addConstraintsToJacobian(constraints entityConstraints1, constraints entityConstraints2, constraints mateConstraints)
        {
            int constrainedDOFs = jointConstraints.constrainedDOFs | ( entityConstraints1.constrainedDOFs & entityConstraints2.constrainedDOFs);

            if ((constrainedDOFs & (int)constraints.ConstrainedDOFs.X) != 0)
            {
                Jacobian = OPS.addVectorToMatrix(Jacobian, OPS.vectorCat(entityConstraints1.X, entityConstraints2.X));
                // After adding constraint, the Jacobian is put into row-reduced echelon form which serves to remove any non-linearly independent rows
                Jacobian = OPS.rref(Jacobian);
            }
            if ((constrainedDOFs & (int)constraints.ConstrainedDOFs.Y) != 0)
            {
                Jacobian = OPS.addVectorToMatrix(Jacobian, OPS.vectorCat(entityConstraints1.Y, entityConstraints2.Y));
                Jacobian = OPS.rref(Jacobian);
            }
            if ((constrainedDOFs & (int)constraints.ConstrainedDOFs.Z) != 0)
            {
                Jacobian = OPS.addVectorToMatrix(Jacobian, OPS.vectorCat(entityConstraints1.Z, entityConstraints2.Z));
                Jacobian = OPS.rref(Jacobian);
            }
            if ((constrainedDOFs & (int)constraints.ConstrainedDOFs.Roll) != 0)
            {
                Jacobian = OPS.addVectorToMatrix(Jacobian, OPS.vectorCat(entityConstraints1.Roll, entityConstraints2.Roll));
                Jacobian = OPS.rref(Jacobian);
            }
            if ((constrainedDOFs & (int)constraints.ConstrainedDOFs.Pitch) != 0)
            {
                Jacobian = OPS.addVectorToMatrix(Jacobian, OPS.vectorCat(entityConstraints1.Pitch, entityConstraints2.Pitch));
                Jacobian = OPS.rref(Jacobian);
            }
            if ((constrainedDOFs & (int)constraints.ConstrainedDOFs.Yaw) != 0)
            {
                Jacobian = OPS.addVectorToMatrix(Jacobian, OPS.vectorCat(entityConstraints1.Yaw, entityConstraints2.Yaw));
                Jacobian = OPS.rref(Jacobian);
            }
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
        public constraints setConstraints(Mate2 mate)
        {
            constraints mateConstraints = new constraints();
            switch (mate.Type)
            {
                case (int)swMateType_e.swMateCOINCIDENT:
                    mateConstraints.constrainedDOFs = ((int)constraints.ConstrainedDOFs.X | (int)constraints.ConstrainedDOFs.Roll | (int)constraints.ConstrainedDOFs.Pitch);
                    break;
                case (int)swMateType_e.swMateCONCENTRIC:
                    mateConstraints.constrainedDOFs = ((int)constraints.ConstrainedDOFs.X | (int)constraints.ConstrainedDOFs.Y | (int)constraints.ConstrainedDOFs.Roll | (int)constraints.ConstrainedDOFs.Pitch);
                    break;
                case (int)swMateType_e.swMatePERPENDICULAR:
                    mateConstraints.constrainedDOFs = ((int)constraints.ConstrainedDOFs.Pitch | (int)constraints.ConstrainedDOFs.Yaw);
                    break;
                case (int)swMateType_e.swMatePARALLEL:
                    mateConstraints.constrainedDOFs = ((int)constraints.ConstrainedDOFs.Roll | (int)constraints.ConstrainedDOFs.Pitch);
                    break;
                case (int)swMateType_e.swMateTANGENT:
                    break;
                case (int)swMateType_e.swMateDISTANCE:
                    mateConstraints.constrainedDOFs = ((int)constraints.ConstrainedDOFs.X | (int)constraints.ConstrainedDOFs.Roll | (int)constraints.ConstrainedDOFs.Pitch);
                    break;
                case (int)swMateType_e.swMateANGLE:
                    mateConstraints.constrainedDOFs = ((int)constraints.ConstrainedDOFs.Roll | (int)constraints.ConstrainedDOFs.Pitch);
                    break;
                case (int)swMateType_e.swMateUNKNOWN:
                    break;
                case (int)swMateType_e.swMateSYMMETRIC:
                    mateConstraints.constrainedDOFs = ((int)constraints.ConstrainedDOFs.X | (int)constraints.ConstrainedDOFs.Roll | (int)constraints.ConstrainedDOFs.Pitch);
                    break;
                case (int)swMateType_e.swMateCAMFOLLOWER:
                    break;
                case (int)swMateType_e.swMateGEAR:
                    break;
                case (int)swMateType_e.swMateWIDTH:
                    mateConstraints.constrainedDOFs = ((int)constraints.ConstrainedDOFs.X | (int)constraints.ConstrainedDOFs.Roll | (int)constraints.ConstrainedDOFs.Pitch);
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
            return mateConstraints;
        }

        public Matrix getRotationMatrix(MathTransform transform)
        {
            var rot = new DenseMatrix(3);
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    rot.At(i, j, transform.ArrayData[i + 3*j]);
            return rot;
        }
    }
}
