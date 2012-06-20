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
using MathNet.Numerics.LinearAlgebra.Double;

namespace SW2URDF
{
    public class MatrixOPS
    {
        public Matrix rref(Matrix m)
        {
            return m;
        }

        public Matrix nullSpace(Matrix m)
        {
            return m;
        }
    }

    public class constraint
    {
        public Vector vec
        { get; set; }
        public bool constrained
        { get; set; }
    }

    public class constraints
    {
        public constraint X
        { get; set; }
        public constraint Y
        { get; set; }
        public constraint Z
        { get; set; }
        public constraint Roll
        { get; set; }
        public constraint Pitch
        { get; set; }
        public constraint Yaw
        { get; set; }

        public void setLinearVector(Vector v)
        {

        }
        public void setTwoLinearVectors(Vector v1, Vector v2)
        {
        }
        public void setThreeLinearVectors(Vector v1, Vector v2, Vector v3)
        {
        }
        public void setRotationVector(Vector v)
        {
        }
        public void setTwoRotationVectors(Vector v1, Vector v2)
        {
        }
        public void setThreeRotationVectors(Vector v1, Vector v2, Vector v3)
        {
        }   
    }

    public class relation
    {
        private Matrix Jacobian;

        public relation()
        {
            Jacobian = new DenseMatrix(12);
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
