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

namespace SW2URDF
{
    public class SW2URDFExporter
    {
        #region Local variables
        ISldWorks iSwApp = null;
        ops OPS;

        public robot mRobot
        { get; set; }
        public string mPackageName
        { get; set; }
        public string mSavePath
        { get; set; }
        private bool mBinary;
        private int mSTLUnits;
        private int mSTLQuality;
        private bool mshowInfo;
        private bool mSTLPreview;
        public List<link> mLinks
        { get; set; }
        ModelDoc2 ActiveSWModel;
        MathUtility swMath;
        #endregion

        public SW2URDFExporter(ISldWorks iSldWorksApp)
        {
            iSwApp = iSldWorksApp;
            ActiveSWModel = default(ModelDoc2);
            ActiveSWModel = (ModelDoc2)iSwApp.ActiveDoc;
            mSavePath = System.Environment.ExpandEnvironmentVariables("%HOMEDRIVE%%HOMEPATH%");
            mPackageName = ActiveSWModel.GetTitle();
            swMath = iSwApp.GetMathUtility();
            OPS = new ops();
        }

        #region SW to Robot methods
        public void createRobotFromActiveModel()
        {
            mRobot = new robot();
            mRobot.name = ActiveSWModel.GetTitle();

            int modelType = ActiveSWModel.GetType();
            if (modelType == (int)swDocumentTypes_e.swDocASSEMBLY)
            {
                mRobot.BaseLink = getBaseLinkFromActiveModel();
            }
            else if (modelType == (int)swDocumentTypes_e.swDocPART)
            {
                mRobot.BaseLink = getLinkFromActiveModel();
            }
        }

        public link getBaseLinkFromActiveModel()
        {
            return getBaseLinkFromAssy(ActiveSWModel);
        }

        public link getBaseLinkFromAssy(ModelDoc2 swModel)
        {
            AssemblyDoc swAssy = (AssemblyDoc)swModel;
            object[] varComp = swAssy.GetComponents(true);

            //For building tree through assembly hierachy (more robust)
            link baseLink = new link();
            foreach (IComponent2 comp in varComp)
            {
                link sparseLink = createSparseBranchFromComponents(comp, 0);
                if (sparseLink != null)
                {
                    baseLink.Children.Add(sparseLink);
                }
            }

            return assignParentLinks(baseLink, 0);
            //link baseLink = assignParentLinks(sparseLink, 0);
            ////For building tree through mate linkages (should give better results when it works)
            //IComponent2 parent = findParent(varComp, 0);
            //List<IComponent2> list = new List<IComponent2>();
            //list.Add(parent);
            //return createBaseComponent(swModel);
        }

        public link getLinkFromPartComp(object comp, int level)
        {
            IComponent2 parentComp = (IComponent2)comp;
            ModelDoc2 parentdoc = parentComp.GetModelDoc();
            link Link;

            if (parentdoc == null)
            {
                throw new System.InvalidOperationException("Component " + parentComp.Name2 + " is null");
            }

            Link = getLinkFromPartModel(parentdoc);
            Link.SWComponent = parentComp;
            Link.SWComponentLevel = level;
            Link.uniqueName = parentComp.Name2;
            return Link;
        }

        public link createLinkFromAttachedLinks(IComponent2 comp, List<IComponent2> matedComponents, int level)
        {
            link Link = getLinkFromPartModel(comp.GetModelDoc2());
            Link.SWComponent = comp;

            object[] mates = comp.GetMates();
            if (mates != null)
            {
                //Dies at level 1 here
                foreach (object mate in mates)
                {
                    if (mate is Mate2)
                    {
                        Mate2 swMate = (Mate2)mate;
                        int type = swMate.Type;
                        int entityCount = swMate.GetMateEntityCount();
                        for (int i = 0; i < entityCount; i++)
                        {
                            MateEntity2 entity = swMate.MateEntity(i);
                            int t = entity.ReferenceType2;
                            IComponent2 entityComponent = default(IComponent2);

                            if (entity.ReferenceComponent != null)
                            {
                                entityComponent = entity.ReferenceComponent;
                            }
                            if (entityComponent != null)
                            {
                                ModelDoc2 model = entityComponent.GetModelDoc2();
                                bool alreadyFound = entity.ReferenceComponent == comp || matedComponents.Contains(entity.ReferenceComponent);
                                if (!alreadyFound)
                                {
                                    matedComponents.Add(entity.ReferenceComponent);
                                    if (model.GetType() == (int)swDocumentTypes_e.swDocPART)
                                    {
                                        Link.Children.Add(createLinkFromAttachedLinks(entity.ReferenceComponent, matedComponents, level + 1));
                                    }
                                }
                            }
                        }
                    }
                    else if (mate is MateInPlace)
                    {
                        int c = 1;
                    }
                }
            }

            return Link;
        }

        public link createBaseComponent(IComponent2 component, List<IComponent2> matedComponents, int level)
        {
            link Link;
            ModelDoc2 model = component.GetModelDoc2();

            IComponent2 parentComp = findParent(component.GetChildren(), level + 1);
            Link = getLinkFromPartModel(parentComp.GetModelDoc2());
            IComponent2 parentAssy = parentComp.GetParent();

            Link.Children.AddRange(createLinksFromMatedComponents(parentComp, model, model, matedComponents, level));
            return Link;
        }

        public link createBaseComponent(ModelDoc2 model)
        {
            link Link;
            List<IComponent2> matedComponents = new List<IComponent2>();
            if (model.GetType() == (int)swDocumentTypes_e.swDocASSEMBLY)
            {
                AssemblyDoc assy = (AssemblyDoc)model;
                IComponent2 parentComp = findParent(assy.GetComponents(false), 0);
                Link = getLinkFromPartModel(parentComp.GetModelDoc2());
                Link.Children.AddRange(createLinksFromMatedComponents(parentComp, model, model, matedComponents, 0));
            }
            else
            {
                Link = getLinkFromPartModel(model);
            }
            IComponent2 comp = Link.SWComponent;
            int SWComponentLevel = Link.SWComponentLevel;
            while (SWComponentLevel > 0)
            {
                IComponent2 parentComp = comp.GetParent();
                ModelDoc2 parentDoc = parentComp.GetModelDoc2();
                Link.Children.AddRange(createLinksFromMatedComponents(comp, parentDoc, model, matedComponents, SWComponentLevel));
                SWComponentLevel--;
            }
            return Link;
        }

        public List<link> createLinksFromMatedComponents(IComponent2 component, ModelDoc2 parentModel, ModelDoc2 rootModel, List<IComponent2> matedComponents, int level)
        {
            List<link> links = new List<link>();
            AssemblyDoc parentAssy = (AssemblyDoc)parentModel;

            int errors = 0;
            iSwApp.ActivateDoc3(parentModel.GetTitle() + ".sldasm", false, (int)swRebuildOnActivation_e.swUserDecision, ref errors);

            string[] names = component.Name2.Split('/');
            string componentName = "";
            if (names[names.Length - 1] != "")
            {
                componentName = names[names.Length - 1];
            }
            else
            {
                componentName = component.Name2;
            }
            IComponent2 componentNew = parentAssy.GetComponentByName(componentName);
            matedComponents.Add(componentNew);
            ModelDoc2 model = componentNew.GetModelDoc2();

            if (componentNew != null && model.GetType() == (int)swDocumentTypes_e.swDocPART)
            {
                object[] mates = componentNew.GetMates();

                foreach (object mate in mates)
                {
                    if (mate is Mate2)
                    {
                        Mate2 swMate = (Mate2)mate;
                        for (int i = 0; i < swMate.GetMateEntityCount(); i++)
                        {
                            MateEntity2 entity = swMate.MateEntity(i);
                            ModelDoc2 entitymodel = entity.ReferenceComponent.GetModelDoc2();

                            if (!(entity.ReferenceComponent == null || matedComponents.Contains(entity.ReferenceComponent) || entity.ReferenceComponent.IsRoot() || entity.ReferenceComponent.IsSuppressed()))
                            {
                                IComponent2 entityComp = entity.ReferenceComponent;
                                matedComponents.Add(entity.ReferenceComponent);

                                link childLink = getLinkFromPartModel(model);
                                childLink.SWComponent = entity.ReferenceComponent;
                                IComponent2 parentAssyComponent = entity.ReferenceComponent.GetParent();
                                ModelDoc2 parentDoc;
                                if (parentAssyComponent != null)
                                {
                                    parentDoc = parentAssyComponent.GetModelDoc2();
                                }
                                else
                                {
                                    parentDoc = parentModel;
                                }
                                List<link> childLinks = createLinksFromMatedComponents(entity.ReferenceComponent, parentDoc, rootModel, matedComponents, level);
                                childLink.Children.AddRange(childLinks);
                                links.Add(childLink);

                            }

                        }
                    }
                }
            }
            return links;
        }

        public link createSparseBranchFromComponents(IComponent2 comp, int level)
        {
            link Link = new link();
            if (comp.IsHidden(true))
            {
                return null;
            }
            if (comp.IGetChildrenCount() == 0)
            {
                Link = getLinkFromPartComp(comp, level);
            }
            else
            {
                object[] children = comp.GetChildren();
                foreach (IComponent2 child in children)
                {
                    link childLink = createSparseBranchFromComponents(child, level + 1);
                    if (childLink != null)
                    {
                        Link.Children.Add(childLink);
                    }
                }
            }
            Link.SWComponent = comp;
            Link.SWComponentLevel = level;
            return Link;
        }

        public link assignParentLinks(link top, int level)
        {
            // For this empty link, find the child link that is the best fit for parenting
            link parentLink = findParent(top, level + 1);

            if (parentLink == null || parentLink.Children.Count == 0)
            {
                return top;
            }
            top = parentLink;
            List<link> linksToRemove = new List<link>();
            List<link> linksToAdd = new List<link>();
            // Iterate through children to continue finding the best parents
            foreach (link child in top.Children)
            {
                // Only bother if the component is not hidden and not supressed
                if (!child.SWComponent.IsHidden(true))
                {
                    linksToAdd.Add(assignParentLinks(child, level + 1));
                    linksToRemove.Add(child);
                }
            }
            // Remove unorganized links
            foreach (link Link in linksToRemove)
            {
                top.Children.Remove(Link);
            }
            // Add organized links
            foreach (link Link in linksToAdd)
            {
                top.Children.Add(Link);
            }
            return top;
        }

        public link findParent(link top, int level)
        {
            if (top.Children.Count > 0)
            {
                link AssignedParentLink = new link();
                int priorityLevel = -1;
                double largestFixedVolume = 0;
                double largestPartVolume = 0;
                double largestAssyVolume = 0;

                // Iterate through children to find the 'best' component for parent status. It may be several assemblies down.
                foreach (link child in top.Children)
                {
                    if (!child.SWComponent.IsHidden(true))
                    {
                        ModelDoc2 ChildDoc = child.SWComponent.GetModelDoc();
                        if (ChildDoc == null)
                        {
                            throw new System.InvalidOperationException("Component " + child.SWComponent.Name2 + " is null");
                        }
                        int ChildType = (int)ChildDoc.GetType();

                        IMassProperty childMass = ChildDoc.Extension.CreateMassProperty();

                        double childVolume = childMass.Volume;

                        //Highest priority is the largest fixed component
                        if (child.SWComponent.IsFixed() && childMass.Volume > largestFixedVolume)
                        {
                            priorityLevel = 2;
                            AssignedParentLink = child;
                            largestFixedVolume = childVolume;
                        }
                        //Second highest priority is the largest floating part
                        else if (childMass.Volume > largestPartVolume && ChildType == (int)swDocumentTypes_e.swDocPART && priorityLevel < 2)
                        {
                            priorityLevel = 1;
                            AssignedParentLink = child;
                            largestPartVolume = childVolume;
                        }
                        //Third priority is the 'best' choice from the largest assembly
                        else if (childMass.Volume > largestAssyVolume && ChildType == (int)swDocumentTypes_e.swDocASSEMBLY && priorityLevel < 1)
                        {
                            priorityLevel = 0;
                            AssignedParentLink = child;
                            largestAssyVolume = childVolume;
                        }
                    }
                }

                ModelDoc2 AssignedParentDoc = AssignedParentLink.SWComponent.GetModelDoc();
                int AssignedParentType = AssignedParentDoc.GetType();
                // If a fixed component was chosen and it is an assembly, iterate through assembly
                if (priorityLevel == 2 && AssignedParentType == (int)swDocumentTypes_e.swDocASSEMBLY)
                {
                    return findParent(AssignedParentLink, level + 1);

                }
                // If no parts were found, iterate through the chosen assembly
                else if (priorityLevel == 0)
                {
                    return findParent(AssignedParentLink, level + 1);
                }
                top.Children.Remove(AssignedParentLink);
                AssignedParentLink.Children.AddRange(top.Children);
                return AssignedParentLink;
            }
            return top;
        }

        public IComponent2 findParent(object[] children, int level)
        {
            IComponent2 AssignedParentComponent = default(IComponent2);
            int priorityLevel = -1;
            double largestFixedVolume = 0;
            double largestPartVolume = 0;
            double largestAssyVolume = 0;

            // Iterate through children to find the 'best' component for parent status. It may be several assemblies down.
            foreach (IComponent2 child in children)
            {
                if (!child.IsHidden(true))
                {
                    ModelDoc2 ChildDoc = child.GetModelDoc();
                    if (ChildDoc == null)
                    {
                        throw new System.InvalidOperationException("Component " + child.Name2 + " is null");
                    }
                    int ChildType = (int)ChildDoc.GetType();

                    IMassProperty childMass = ChildDoc.Extension.CreateMassProperty();

                    double childVolume = childMass.Volume;

                    //Highest priority is the largest fixed component
                    if (child.IsFixed() && childMass.Volume > largestFixedVolume)
                    {
                        priorityLevel = 2;
                        AssignedParentComponent = child;
                        largestFixedVolume = childVolume;
                    }
                    //Second highest priority is the largest floating part
                    else if (childMass.Volume > largestPartVolume && ChildType == (int)swDocumentTypes_e.swDocPART && priorityLevel < 2)
                    {
                        priorityLevel = 1;
                        AssignedParentComponent = child;
                        largestPartVolume = childVolume;
                    }
                    //Third priority is the 'best' choice from the largest assembly
                    else if (childMass.Volume > largestAssyVolume && ChildType == (int)swDocumentTypes_e.swDocASSEMBLY && priorityLevel < 1)
                    {
                        priorityLevel = 0;
                        AssignedParentComponent = child;
                        largestAssyVolume = childVolume;
                    }
                }
            }

            ModelDoc2 AssignedParentDoc = AssignedParentComponent.GetModelDoc();
            int AssignedParentType = AssignedParentDoc.GetType();
            // If a fixed component was chosen and it is an assembly, iterate through assembly
            if (priorityLevel == 2 && AssignedParentType == (int)swDocumentTypes_e.swDocASSEMBLY)
            {
                return findParent(AssignedParentComponent.GetChildren(), level + 1);

            }
            // If no parts were found, iterate through the chosen assembly
            else if (priorityLevel == 0)
            {
                return findParent(AssignedParentComponent.GetChildren(), level + 1);
            }

            return AssignedParentComponent;

        }
        #endregion

        #region Joint methods
        public void createJoints()
        {
            mRobot.BaseLink = createChildLinks(mRobot.BaseLink);
            foreach (link child in mRobot.BaseLink.Children)
            {
                adjustJointTransforms(child, DenseMatrix.Identity(4));
            }
        }

        public void adjustJointTransforms(link Link, Matrix<double> cumulativeTransform)
        {
            Vector<double> Axis = new DenseVector(new double[4]{Link.Joint.Axis.X, Link.Joint.Axis.Y, Link.Joint.Axis.Z, 0});
            double[] XYZ = OPS.getXYZ(Link.SWComponent.Transform2);
            Matrix<double> meshTransform = OPS.getTransformation(Link.SWComponent.Transform2);
            Vector<double> Point = new DenseVector(new double[] { Link.Joint.Origin.X, Link.Joint.Origin.Y, Link.Joint.Origin.Z, 1 });
            Vector<double> Origin = new DenseVector(new double[]{XYZ[0], XYZ[1], XYZ[2], 1});
            Matrix<double> newTransform = OPS.getTransformation(Link.Joint.Origin.XYZ, Link.Joint.Origin.RPY);
            Matrix<double> localTransform = newTransform * cumulativeTransform.Inverse();
            Axis = newTransform.Inverse() * Axis;
            Point = cumulativeTransform.Inverse() * Point;
            Link.Joint.Axis.XYZ = new double[] { Axis[0], Axis[1], Axis[2] };
            
            Origin = newTransform.Inverse() * Origin;
            Link.Joint.Origin.XYZ = new double[] { Point[0], Point[1], Point[2] };



            double[] xyz_mesh = OPS.getXYZ(meshTransform);
            double[] rpy_mesh = OPS.getRPY(meshTransform);
            meshTransform = meshTransform * newTransform.Inverse();
            double[] xyz_mafter = OPS.getXYZ(meshTransform);
            double[] rpy_mafter = OPS.getRPY(meshTransform);

            double[] rpy_new = OPS.getRPY(newTransform);
            double[] rpy_cum = OPS.getRPY(cumulativeTransform);
            double[] rpy_cum_inv = OPS.getRPY(cumulativeTransform.Inverse());
            double[] xyz_new = OPS.getXYZ(newTransform);
            double[] xyz_cum = OPS.getXYZ(cumulativeTransform);
            double[] xyz_cum_inv = OPS.getXYZ(cumulativeTransform.Inverse());
            Link.Joint.Origin.RPY = OPS.getRPY(meshTransform);

            double[] data1 = Link.SWComponent.Transform2.ArrayData;
            Matrix<double> linkTransform = OPS.getTransformation(Link.SWComponent.Transform2);
            Matrix<double> localLinkTransform = linkTransform * newTransform.Inverse();


            Link.Visual.Origin.XYZ = OPS.getXYZ(meshTransform);
            Link.Visual.Origin.RPY = OPS.getRPY(meshTransform);
            Link.Inertial.Origin.XYZ = Link.Visual.Origin.XYZ;
            Link.Inertial.Origin.RPY = Link.Visual.Origin.RPY;
            Link.Collision.Origin.XYZ = Link.Visual.Origin.XYZ;
            Link.Collision.Origin.RPY = Link.Visual.Origin.RPY;

            foreach (link child in Link.Children)
            {
                adjustJointTransforms(child, newTransform);
            }


           // return Link.Joint;
        }

        public link createChildLinks(link parent)
        {
            foreach (link child in parent.Children)
            {
                child.Joint = createJointFromLinks(parent, child);
                createChildLinks(child);
            }
            return parent;
        }

        public joint createJointFromLinks(link parent, link child)
        {
            object[] mates = parent.SWComponent.GetMates();


            //jointEstimation estimation = new jointEstimation(iSwApp);
            //joint Joint = estimation.estimateJointFromComponents((AssemblyDoc)ActiveSWModel, parent.SWComponent, child.SWComponent, true);
            joint Joint = estimateJointFromComponents((AssemblyDoc)ActiveSWModel, parent.SWComponent, child.SWComponent);
            Joint.name = parent.uniqueName + "_to_" + child.uniqueName;

            Joint.Parent.name = parent.uniqueName;
            Joint.Child.name = child.uniqueName;

            Joint.Dynamics.friction = 0;
            Joint.Dynamics.damping = 0;

            Joint.Calibration.rising = 0;
            Joint.Calibration.falling = 0;

            Joint.Limit.upper = 0;
            Joint.Limit.lower = 0;
            Joint.Limit.effort = 0;
            Joint.Limit.velocity = 0;

            Joint.Safety.soft_upper = 0;
            Joint.Safety.soft_lower = 0;
            Joint.Safety.k_velocity = 0;
            Joint.Safety.k_position = 0;

            return Joint;
        }

        public joint estimateJointFromComponents(AssemblyDoc assy, IComponent2 parent, IComponent2 child)
        {
            joint Joint = new joint();
            int R1Status, R2Status, L1Status, L2Status;
            int R1DirStatus, R2DirStatus;
            MathPoint RPoint1, RPoint2;
            MathVector RDir1, RDir2;
            MathVector LDir1, LDir2;


            bool isFixed = parent.IsFixed();
            parent.Select(false);
            assy.FixComponent();
            int DOFs = child.GetRemainingDOFs(out R1Status, out RPoint1, out R1DirStatus, out RDir1,
                                              out R2Status, out RPoint2, out R2DirStatus, out RDir2,
                                              out L1Status, out LDir1,
                                              out L2Status, out LDir2);

            parent.Select(false);
            if (!isFixed)
            {
                assy.UnfixComponent();
            }

            if (DOFs == 0 && (R1Status + L1Status > 0))
            {
                if (R1Status == 1 && L1Status == 1)
                {
                    Joint.type = "fixed";
                    MathTransform transform = child.Transform2;
                    Joint.Origin.XYZ = new double[] { transform.ArrayData[9], transform.ArrayData[10], transform.ArrayData[11] };
                    Joint.Origin.RPY = OPS.getRPY(transform);
                }
                else if (R1Status == 1)
                {
                    Joint.type = "continuous";
                    Joint.Axis.XYZ = RDir1.ArrayData;
                    Joint.Origin.XYZ = RPoint1.ArrayData;
                    MathTransform transform = parent.Transform2;
                    Joint.Origin.RPY = OPS.getRPY(transform);
                }
                else if (L1Status == 1)
                {
                    Joint.type = "prismatic";
                    Joint.Axis.XYZ = LDir1.ArrayData;
                    MathTransform transform = child.Transform2;
                    Joint.Origin.XYZ = new double[] { transform.ArrayData[9], transform.ArrayData[10], transform.ArrayData[11] };
                    Joint.Origin.RPY = OPS.getRPY(parent.Transform2);
                }
            }
            else
            {
                Joint.type = "fixed";
                MathTransform transform = child.Transform2;
                Joint.Origin.XYZ = new double[] { transform.ArrayData[9], transform.ArrayData[10], transform.ArrayData[11] };
                Joint.Origin.RPY = OPS.getRPY(parent.Transform2);
            }

            //[TODO] joint comp here...

            return Joint;
        }
        #endregion

        #region Export Methods

        //Copy and export textures here

        public void exportRobot()
        {
            //Creating package directories
            URDFPackage package = new URDFPackage(mPackageName, mSavePath);
            package.createDirectories();
            string windowsURDFFileName = package.WindowsRobotsDirectory + mRobot.BaseLink.name + ".URDF";

            //Customizing STL preferences to how I want them
            saveUserPreferences();
            setSTLExportPreferences();

            //Saving part as STL mesh
            hideComponents(mRobot.BaseLink);
            string filename = exportFiles(mRobot.BaseLink, package);
            showComponents(mRobot.BaseLink);
            mRobot.BaseLink.Visual.Geometry.Mesh.filename = filename;
            mRobot.BaseLink.Collision.Geometry.Mesh.filename = filename;

            //Writing URDF to file
            URDFWriter uWriter = new URDFWriter(windowsURDFFileName);
            mRobot.writeURDF(uWriter.writer);

            resetUserPreferences();
        }

        public string exportFiles(link Link, URDFPackage package)
        {
            foreach (link child in Link.Children)
            {
                string filename = exportFiles(child, package);
                child.Visual.Geometry.Mesh.filename = filename;
                child.Collision.Geometry.Mesh.filename = filename;
            }
            if (Link.Visual.Material.Texture.wFilename != "")
            {
                if (!System.IO.File.Exists(Link.Visual.Material.Texture.wFilename))
                {
                    Link.Visual.Material.Texture.filename = package.TexturesDirectory + Path.GetFileName(Link.Visual.Material.Texture.wFilename);
                    string textureSavePath = package.WindowsTexturesDirectory + Path.GetFileName(Link.Visual.Material.Texture.wFilename);
                    System.IO.File.Copy(Link.Visual.Material.Texture.wFilename, textureSavePath, true);
                }
            }

            string linkName = Link.uniqueName.Replace('/', '_');
            string meshFileName = package.MeshesDirectory + linkName + ".STL";
            string windowsMeshFileName = package.WindowsMeshesDirectory + linkName + ".STL";

            int errors = 0;
            int warnings = 0;

            if (!System.IO.File.Exists(windowsMeshFileName))
            {
                Link.SWComponent.Select(false);
                ActiveSWModel.ShowComponent2();
                //ModelDoc2 modeldoc = Link.SWComponent.GetModelDoc2();

                //iSwApp.ActivateDoc3(Link.name + ".sldprt", false, (int)swRebuildOnActivation_e.swUserDecision, ref errors);
                //modeldoc = iSwApp.IActiveDoc2;
                int saveOptions = (int)swSaveAsOptions_e.swSaveAsOptions_Silent;
                ActiveSWModel.Extension.SaveAs(windowsMeshFileName, (int)swSaveAsVersion_e.swSaveAsCurrentVersion, saveOptions, null, ref errors, ref warnings);
                //iSwApp.CloseDoc(Link.name + ".sldprt");
                Link.SWComponent.Select(false);
                ActiveSWModel.HideComponent2();

                correctSTLMesh(windowsMeshFileName);
            }
            return meshFileName;
        }
        public void selectComponents(link Link)
        {
            Link.SWComponent.Select(true);
            foreach (link child in Link.Children)
            {
                selectComponents(child);
            }
        }
        public void hideComponents(link Link)
        {
            selectComponents(Link);
            ActiveSWModel.HideComponent2();
            
        }
        public void showComponents(link Link)
        {
            selectComponents(Link);
            ActiveSWModel.ShowComponent2();
            
        }

        public void correctSTLMesh(string filename)
        {
            FileStream fileStream = new FileStream(filename, FileMode.OpenOrCreate, FileAccess.ReadWrite, FileShare.None);
            byte[] emptyHeader = new byte[80];
            fileStream.Write(emptyHeader, 0, emptyHeader.Length);
            fileStream.Close();
        }
        #endregion

        #region Part Exporting methods
        public link getLinkFromPartModel(ModelDoc2 swModel)
        {
            link Link = new link();
            Link.name = swModel.GetTitle();


            //Get link properties from SolidWorks part
            IMassProperty swMass = swModel.Extension.CreateMassProperty();
            Link.Inertial.Mass.Value = swMass.Mass;

            Link.Inertial.Inertia.Moment = swMass.GetMomentOfInertia((int)swMassPropertyMoment_e.swMassPropertyMomentAboutCenterOfMass); // returned as double with values [Lxx, Lxy, Lxz, Lyx, Lyy, Lyz, Lzx, Lzy, Lzz]

            double[] centerOfMass = swMass.CenterOfMass;
            Link.Inertial.Origin.XYZ = centerOfMass;
            Link.Inertial.Origin.RPY = new double[3] { 0, 0, 0 };
            Link.Visual.Origin.XYZ = centerOfMass;
            Link.Visual.Origin.RPY = new double[3] { 0, 0, 0 };

            Link.Collision.Origin.XYZ = centerOfMass;
            Link.Collision.Origin.RPY = new double[3] { 0, 0, 0 };

            // [ R, G, B, Ambient, Diffuse, Specular, Shininess, Transparency, Emission ]
            double[] values = swModel.MaterialPropertyValues;
            Link.Visual.Material.Color.Red = values[0];
            Link.Visual.Material.Color.Green = values[1];
            Link.Visual.Material.Color.Blue = values[2];
            Link.Visual.Material.Color.Alpha = 1.0 - values[7];
            Link.Visual.Material.name = "material_" + Link.name;

            return Link;
        }

        public link getLinkFromActiveModel()
        {
            return getLinkFromPartModel(ActiveSWModel);
        }


        public void exportLink()
        {
            //Creating package directories
            URDFPackage package = new URDFPackage(mPackageName, mSavePath);
            package.createDirectories();
            string meshFileName = package.MeshesDirectory + mRobot.BaseLink.name + ".STL";
            string windowsMeshFileName = package.WindowsMeshesDirectory + mRobot.BaseLink.name + ".STL";
            string windowsURDFFileName = package.WindowsRobotsDirectory + mRobot.BaseLink.name + ".URDF";

            //Customizing STL preferences to how I want them
            saveUserPreferences();
            setSTLExportPreferences();
            int errors = 0;
            int warnings = 0;

            //Saving part as STL mesh
            ActiveSWModel.Extension.SaveAs(windowsMeshFileName, (int)swSaveAsVersion_e.swSaveAsCurrentVersion, (int)swSaveAsOptions_e.swSaveAsOptions_Silent, null, ref errors, ref warnings);
            mRobot.BaseLink.Visual.Geometry.Mesh.filename = meshFileName;
            mRobot.BaseLink.Collision.Geometry.Mesh.filename = meshFileName;

            correctSTLMesh(windowsMeshFileName);

            mRobot.BaseLink.Visual.Material.Texture.filename = package.TexturesDirectory + Path.GetFileName(mRobot.BaseLink.Visual.Material.Texture.wFilename);
            string textureSavePath = package.WindowsTexturesDirectory + Path.GetFileName(mRobot.BaseLink.Visual.Material.Texture.wFilename);
            if (mRobot.BaseLink.Visual.Material.Texture.wFilename != "")
            {
                System.IO.File.Copy(mRobot.BaseLink.Visual.Material.Texture.wFilename, textureSavePath, true);
            }

            //Writing URDF to file
            URDFWriter uWriter = new URDFWriter(windowsURDFFileName);
            //mRobot.addLink(mLink);
            mRobot.writeURDF(uWriter.writer);

            resetUserPreferences();
        }
        #endregion

        #region STL Preference shuffling
        public void saveUserPreferences()
        {
            mBinary = iSwApp.GetUserPreferenceToggle((int)swUserPreferenceToggle_e.swSTLBinaryFormat);
            mSTLUnits = iSwApp.GetUserPreferenceIntegerValue((int)swUserPreferenceIntegerValue_e.swExportStlUnits);
            mSTLQuality = iSwApp.GetUserPreferenceIntegerValue((int)swUserPreferenceIntegerValue_e.swSTLQuality);
            mshowInfo = iSwApp.GetUserPreferenceToggle((int)swUserPreferenceToggle_e.swSTLShowInfoOnSave);
            mSTLPreview = iSwApp.GetUserPreferenceToggle((int)swUserPreferenceToggle_e.swSTLPreview);
        }

        public void setSTLExportPreferences()
        {
            iSwApp.SetUserPreferenceToggle((int)swUserPreferenceToggle_e.swSTLBinaryFormat, true);
            iSwApp.SetUserPreferenceToggle((int)swUserPreferenceToggle_e.swSTLDontTranslateToPositive, true);
            iSwApp.SetUserPreferenceIntegerValue((int)swUserPreferenceIntegerValue_e.swExportStlUnits, 2);
            iSwApp.SetUserPreferenceIntegerValue((int)swUserPreferenceIntegerValue_e.swSTLQuality, (int)swSTLQuality_e.swSTLQuality_Coarse);
            iSwApp.SetUserPreferenceToggle((int)swUserPreferenceToggle_e.swSTLShowInfoOnSave, false);
            iSwApp.SetUserPreferenceToggle((int)swUserPreferenceToggle_e.swSTLPreview, false);
        }

        public void resetUserPreferences()
        {
            iSwApp.SetUserPreferenceToggle((int)swUserPreferenceToggle_e.swSTLBinaryFormat, mBinary);
            iSwApp.SetUserPreferenceIntegerValue((int)swUserPreferenceIntegerValue_e.swExportStlUnits, mSTLUnits);
            iSwApp.SetUserPreferenceIntegerValue((int)swUserPreferenceIntegerValue_e.swSTLQuality, mSTLQuality);
            iSwApp.SetUserPreferenceToggle((int)swUserPreferenceToggle_e.swSTLShowInfoOnSave, mshowInfo);
            iSwApp.SetUserPreferenceToggle((int)swUserPreferenceToggle_e.swSTLPreview, mSTLPreview);
        }
        #endregion

        #region Mates to DOF to Joints types



        #endregion
    }
}