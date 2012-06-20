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

namespace SW2URDF
{
    public class dof
    {
        public bool constrained
        { get; set; }
        public double upper_limit
        { get; set; }
        public double lower_limit
        { get; set; }
    }
    public class relation
    {
        public dof X
        { get; set; }
        public dof Y
        { get; set; }
        public dof Z
        { get; set; }
        public dof Roll
        { get; set; }
        public dof Pitch
        { get; set; }
        public dof Yaw
        { get; set; }

        public void constrainRelationFromMate(Mate2 mate, IComponent2 comp1, IComponent2 comp2)
        {
            switch (mate.Type)
            {
                case (int)swMateType_e.swMateCOINCIDENT:
                    MateEntity2 entity1 = mate.MateEntity(0);
                    MateEntity2 entity2 = mate.MateEntity(1);
                    constrainRelation(entity1, entity2);

                    for (int i = 0; i < mate.GetMateEntityCount(); i++)
                    {
                        MateEntity2 entity = mate.MateEntity(i);
                        entity.ReferenceType2
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
