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
    // The constraint class represents a 6-DOF constraint. Each dof can be represented by a single axis. This doesn't support non-linear contraints (I.E movements constrained in circles, parabolas, screws, etc...), yet.
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

        [Flags]
        public enum ConstrainedDOFs
        {
            X = 1,
            Y = 2,
            Z = 4,
            XYZ = X | Y | Z,
            Roll = 8,
            Pitch = 16,
            Yaw = 32,
            RPY = Roll | Pitch | Yaw,
        }
        public constraints()
        {
            constrainedDOFs = 0;
        }

    }

    // There has to be a better name for what this class does, but for now it will stay as its named. The purpose of this class is to convert SolidWorks mates and their entities into
    // allowable degrees of freedom for the joint. From this information you can get the type, location, axis and hopefully limits for the joint
    public class relation
    {
        private Matrix Jacobian;
        private Matrix JacobianNullSpace;
        private constraints jointConstraints;
        private ops OPS;


        private Vector axis1 = new DenseVector(new double[3] { 1, 0, 0 });
        private Vector axis2 = new DenseVector(new double[3] { 0, 1, 0 });
        private Vector axis3 = new DenseVector(new double[3] { 0, 0, 1 });

        public relation()
        {
            Jacobian = new DenseMatrix(12);
            Jacobian.SetSubMatrix(0, 6, 0, 6, DenseMatrix.Identity(6));
            OPS = new ops();
            jointConstraints = new constraints();
            jointConstraints.constrainedDOFs = 0;
        }

        // This method adds constraints to the Jacobian based on the the mate type, and mating entities
        public void constrainRelationFromMate(Mate2 mate, IComponent2 comp1, IComponent2 comp2)
        {
            if (mate.GetMateEntityCount() == 2)
            {
                // Set constraints given by the two mating entities
                constraints entityConstraints1 = setConstraints(mate.MateEntity(0));
                constraints entityConstraints2 = setConstraints(mate.MateEntity(1));

                // Set constraints given by the mate itself
                constraints mateConstraints = setConstraints(mate);

                //Add these constraits to the Jacobian matrix
                addConstraintsToJacobian(entityConstraints1, entityConstraints2, mateConstraints);
            }
            else
            {
                switch (mate.Type)
                {
                    case (int)swMateType_e.swMateCOINCIDENT:
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
        }

        // This method adds the constrained degrees of freedom to the Jacobian based on the constraints from each entity and the mate constraints.
        public void addConstraintsToJacobian(constraints entityConstraints1, constraints entityConstraints2, constraints mateConstraints)
        {

            int constrainedDOFs = (entityConstraints1.constrainedDOFs & entityConstraints2.constrainedDOFs & mateConstraints.constrainedDOFs);

            Vector Zeros3 = (DenseVector)new double[3] { 0, 0, 0 };
            if ((constrainedDOFs & (int)constraints.ConstrainedDOFs.X) != 0)
            {
                Vector v = OPS.vectorCat(entityConstraints1.X, Zeros3, entityConstraints2.X, Zeros3);
                Jacobian = OPS.addVectorToMatrix(Jacobian, v);
                // After adding constraint, the Jacobian is put into row-reduced echelon form which serves to remove any non-linearly independent rows
                Jacobian = OPS.rref(Jacobian);
            }
            if ((constrainedDOFs & (int)constraints.ConstrainedDOFs.Y) != 0)
            {
                Vector v = OPS.vectorCat(entityConstraints1.Y, Zeros3, entityConstraints2.Y, Zeros3);
                Jacobian = OPS.addVectorToMatrix(Jacobian, v);
                Jacobian = OPS.rref(Jacobian);
            }
            if ((constrainedDOFs & (int)constraints.ConstrainedDOFs.Z) != 0)
            {
                Vector v = OPS.vectorCat(entityConstraints1.Z, Zeros3, entityConstraints2.Z, Zeros3);
                Jacobian = OPS.addVectorToMatrix(Jacobian, v);
                Jacobian = OPS.rref(Jacobian);
            }
            if ((constrainedDOFs & (int)constraints.ConstrainedDOFs.Roll) != 0)
            {
                Vector v = OPS.vectorCat(Zeros3, entityConstraints1.Roll, Zeros3, entityConstraints2.Roll);
                Jacobian = OPS.addVectorToMatrix(Jacobian, v);
                Jacobian = OPS.rref(Jacobian);
            }
            if ((constrainedDOFs & (int)constraints.ConstrainedDOFs.Pitch) != 0)
            {
                Vector v = OPS.vectorCat(Zeros3, entityConstraints1.Pitch, Zeros3, entityConstraints2.Pitch);
                Jacobian = OPS.addVectorToMatrix(Jacobian, v);
                Jacobian = OPS.rref(Jacobian);
            }
            if ((constrainedDOFs & (int)constraints.ConstrainedDOFs.Yaw) != 0)
            {
                Vector v = OPS.vectorCat(Zeros3, entityConstraints1.Yaw, Zeros3, entityConstraints2.Yaw);
                Jacobian = OPS.addVectorToMatrix(Jacobian, v);
                Jacobian = OPS.rref(Jacobian);
            }
            jointConstraints.constrainedDOFs |= constrainedDOFs;
            JacobianNullSpace = OPS.nullSpace(Jacobian);
        }

        // This method returns the constraints derived from the mating entity. For example a point is constrained in three linear directions but no rotational directions.
        public constraints setConstraints(MateEntity2 entity)
        {
            if (entity.ReferenceComponent == null)
            {
                return null;
            }

            // ReferenceType is obsoleted by ReferenceType2 but they reference different enumerations and I don't know why
            // The first one makes things simpler (I think).
            switch (entity.ReferenceType)
            {
                case (int)swMateEntity2ReferenceType_e.swMateEntity2ReferenceType_Point:
                    return setPointConstraints(entity);
                case (int)swMateEntity2ReferenceType_e.swMateEntity2ReferenceType_Line:
                    return setLineOrCylinderConstraints(entity);             
                case (int)swMateEntity2ReferenceType_e.swMateEntity2ReferenceType_Circle:
                    return setCircleConstraints(entity);
                case (int)swMateEntity2ReferenceType_e.swMateEntity2ReferenceType_Plane:
                    return setPlaneConstraints(entity);
                case (int)swMateEntity2ReferenceType_e.swMateEntity2ReferenceType_Cylinder:
                    return setLineOrCylinderConstraints(entity);
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
            return null;
        }

        // This method returns the maximum possible constraints derived from the mate itself. For example a concentric mate can constrain two entities in three linear directions and two rotational directions (two spheres are constrained like a point, a cylinder like a cylinder).
        public constraints setConstraints(Mate2 mate)
        {
            constraints mateConstraints = new constraints();
            switch (mate.Type)
            {
                case (int)swMateType_e.swMateCOINCIDENT:
                    mateConstraints.constrainedDOFs = ((int)constraints.ConstrainedDOFs.X | (int)constraints.ConstrainedDOFs.Roll | (int)constraints.ConstrainedDOFs.Pitch);
                    break;
                case (int)swMateType_e.swMateCONCENTRIC:
                    mateConstraints.constrainedDOFs = ((int)constraints.ConstrainedDOFs.X | (int)constraints.ConstrainedDOFs.Y | (int)constraints.ConstrainedDOFs.Y | (int)constraints.ConstrainedDOFs.Roll | (int)constraints.ConstrainedDOFs.Pitch);
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

        // A point is constrained in three linear directions. They do not need to be transformed because they are orthogonal
        public constraints setPointConstraints(MateEntity2 entity)
        {
            constraints entityConstraints = new constraints();
            entityConstraints.constrainedDOFs = ((int)constraints.ConstrainedDOFs.X | (int)constraints.ConstrainedDOFs.Y | (int)constraints.ConstrainedDOFs.Z);
            entityConstraints.X = axis1;
            entityConstraints.Y = axis2;
            entityConstraints.Z = axis3;
            return entityConstraints;
        }

        // A line or cylinder have two linear constraints orthogonal to and two rotational constraints around the axis of rotation
        public constraints setLineOrCylinderConstraints(MateEntity2 entity)
        {
            double[] entityParams = entity.EntityParams;
            Vector position = new DenseVector(new double[] { entityParams[0], entityParams[1], entityParams[2] });
            Vector axis = new DenseVector(new double[] { entityParams[3], entityParams[4], entityParams[5] });

            constraints entityConstraints = new constraints();
            entityConstraints.constrainedDOFs = ((int)constraints.ConstrainedDOFs.X | (int)constraints.ConstrainedDOFs.Y | (int)constraints.ConstrainedDOFs.Roll | (int)constraints.ConstrainedDOFs.Pitch);

            // Get some unit vector on the plane defined by the line/cylinder axis
            Vector projectedLine = OPS.projectLineToPlane(axis, axis1);
            if (projectedLine.SumMagnitudes() == 0)
            {
                entityConstraints.X = axis2;
            }
            else
            {
                entityConstraints.X = (DenseVector)projectedLine.Normalize(2);
            }

            // The cross product of the line vector with the first orthogonal constraint yields the third orthogonal vector
            entityConstraints.Y = OPS.crossProduct3(axis, entityConstraints.X);
            entityConstraints.Roll = entityConstraints.X;
            entityConstraints.Pitch = entityConstraints.Y;
            return entityConstraints;
        }

        // A circle is constrained in three directions and has rotation constrained in two directions
        public constraints setCircleConstraints(MateEntity2 entity)
        {
            double[] entityParams = entity.EntityParams;
            Vector position = new DenseVector(new double[] { entityParams[0], entityParams[1], entityParams[2] });
            Vector normal = new DenseVector(new double[] { entityParams[3], entityParams[4], entityParams[5] });

            constraints entityConstraints = new constraints();
            entityConstraints.constrainedDOFs = ((int)constraints.ConstrainedDOFs.X | (int)constraints.ConstrainedDOFs.Y | (int)constraints.ConstrainedDOFs.Z | (int)constraints.ConstrainedDOFs.Roll | (int)constraints.ConstrainedDOFs.Pitch);
            entityConstraints.X = axis1;
            entityConstraints.Y = axis2;
            entityConstraints.Z = axis3;

            // Get some unit vector on the plane the circle lies in
            Vector projectedLine = OPS.projectLineToPlane(normal, axis1);
            if (projectedLine.SumMagnitudes() == 0)
            {
                entityConstraints.Roll = axis2;
            }
            else
            {
                entityConstraints.Roll = (DenseVector)projectedLine.Normalize(2);
            }

            // The cross product of the normal and the roll constaint yied the pitch constraint
            entityConstraints.Pitch = OPS.crossProduct3(normal, entityConstraints.Roll);
            return entityConstraints;
        }

        // A plane has one linear constraint along the plane normal and two rotational constraints around axes that are orthogonal to the plane normal
        public constraints setPlaneConstraints(MateEntity2 entity)
        {
            double[] entityParams = entity.EntityParams;
            Vector position = new DenseVector(new double[] { entityParams[0], entityParams[1], entityParams[2] });
            Vector normal = new DenseVector(new double[] { entityParams[3], entityParams[4], entityParams[5] });

            constraints entityConstraints = new constraints();
            entityConstraints.constrainedDOFs = ((int)constraints.ConstrainedDOFs.X | (int)constraints.ConstrainedDOFs.Roll | (int)constraints.ConstrainedDOFs.Pitch);
            entityConstraints.X = normal;

            Vector projectedLine = OPS.projectLineToPlane(normal, axis1);
            if (projectedLine.SumMagnitudes() == 0)
            {
                entityConstraints.Roll = axis2;
            }
            else
            {
                entityConstraints.Roll = (DenseVector)projectedLine.Normalize(2);
            }
            entityConstraints.Pitch = OPS.crossProduct3(normal, entityConstraints.Roll);
            return entityConstraints;
        }

        // Used to get the 3x3 rotation matrix from a SolidWorks MathTransform
        public Matrix getRotationMatrix(MathTransform transform)
        {
            var rot = new DenseMatrix(3);
            //transform.
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    rot.At(i, j, transform.ArrayData[i + 3 * j]);
            return rot;
        }

        public Vector MathVectorToVector(MathVector mVector)
        {
            // I'm pretty sure the length of the MathVector is always 3
            Vector v = new DenseVector(3);
            for (int i = 0; i < 3; i++)
            {
                v[i] = mVector.ArrayData[i];
            }
            return v;
        }

        public int getNumberOfFreeDOFs()
        {
            JacobianNullSpace = OPS.nullSpace(Jacobian);
            Vector Zeros = new DenseVector(JacobianNullSpace.RowCount);
            int freeDOFs = 0;
            for (int i = 0; i < JacobianNullSpace.ColumnCount; i++)
            {
                bool isZeros = true;
                Vector column = (DenseVector)JacobianNullSpace.Column(i);

                //There has to be a better way to check if this column vector is just zeros.
                for (int j = 0; j < column.Count; j++)
                {
                    if (column[j] != 0)
                    {
                        isZeros = false;
                        break;
                    }
                }
                if (!isZeros)
                {
                    freeDOFs++;
                }
            }
            return freeDOFs;
        }

        public int getFreeDOFColumnNumber()
        {
            Vector Zeros = new DenseVector(JacobianNullSpace.RowCount);
            for (int i = 0; i < JacobianNullSpace.ColumnCount; i++)
            {
                if (JacobianNullSpace.Column(i) != Zeros)
                {
                    return i;
                }
            }
            return -1;
        }

        public double[] getAxesValues(int number)
        {
            if (number >= 0 && number < 3)
            {
                Vector v = (DenseVector)JacobianNullSpace.Column(number);
                return new double[3] { v[0], v[1], v[2] };
            }
            else if (number < 6)
            {
                Vector v = (DenseVector)JacobianNullSpace.Column(number);
                return new double[3] { v[3], v[4], v[5] };
            }
            else
            {
                return new double[3] { 0, 0, 0 };
            }
        }
    }
}
