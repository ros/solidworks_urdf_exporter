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
    public class SW2URDFExporter
    {
        #region Local variables
        ISldWorks iSwApp = null;

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
        #endregion

        public SW2URDFExporter(ISldWorks iSldWorksApp)
        {
            iSwApp = iSldWorksApp;
            ActiveSWModel = default(ModelDoc2);
            ActiveSWModel = (ModelDoc2)iSwApp.ActiveDoc;
            mSavePath = System.Environment.ExpandEnvironmentVariables("%HOMEDRIVE%%HOMEPATH%");
            mPackageName = ActiveSWModel.GetTitle();
            MathUtility math = iSwApp.GetMathUtility();
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

            link baseLink = new link();
            //link baseLink = assignParentLinks(sparseLink, 0);
            //foreach (IComponent2 comp in varComp)
            //{
            //    link sparseLink = createSparseBranchFromComponents(comp, 0);
            //    if (sparseLink != null)
            //    {
            //        baseLink.Children.Add(sparseLink);
            //    }
            //}

            //return assignParentLinks(baseLink, 0);
            IComponent2 parent = findParent(varComp, 0);
            List<IComponent2> list = new List<IComponent2>();
            list.Add(parent);
            return createBaseComponent(swModel);
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

            Link.Children.AddRange(createLinksFromMatedComponents(parentComp, model, matedComponents, level));
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
                Link.Children.AddRange(createLinksFromMatedComponents(parentComp, model, matedComponents, 0));
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
                Link.Children.AddRange(createLinksFromMatedComponents(comp, parentDoc, matedComponents, SWComponentLevel));
                SWComponentLevel--;
            }
            return Link;
        }

        public List<link> createLinksFromMatedComponents(IComponent2 component, ModelDoc2 parentModel, List<IComponent2> matedComponents, int level)
        {
            List<link> links = new List<link>();
            AssemblyDoc parentAssy = (AssemblyDoc)parentModel;
            ModelDoc2 model = component.GetModelDoc2();
            int errors = 0;
            iSwApp.ActivateDoc3(parentModel.GetTitle() + ".sldasm", false, (int)swRebuildOnActivation_e.swUserDecision, ref errors);

            IComponent2 componentNew = parentAssy.GetComponentByName(component.Name2);
            object[] mates = componentNew.GetMates();
            if (mates != null)
            {
                foreach (object mate in mates)
                {
                    if (mate is Mate2)
                    {
                        Mate2 swMate = (Mate2)mate;
                        for (int i = 0; i < swMate.GetMateEntityCount(); i++)
                        {
                            MateEntity2 entity = swMate.MateEntity(i);
                            IComponent2 entityComponent = default(IComponent2);
                            if (entity.ReferenceComponent != null)
                            {
                                entityComponent = entity.ReferenceComponent;
                            }
                            if (entityComponent != null)
                            {
                                ModelDoc2 entitymodel = entityComponent.GetModelDoc2();
                                if (!matedComponents.Contains(entity.ReferenceComponent))
                                {
                                    matedComponents.Add(entity.ReferenceComponent);
                                    if (model.GetType() == (int)swDocumentTypes_e.swDocPART)
                                    {
                                        link childLink = getLinkFromPartModel(model);
                                        
                                        childLink.Children.AddRange(createLinksFromMatedComponents(entityComponent, entityComponent.GetParent().GetModelDoc2(),matedComponents, level + 1));
                                        links.Add(childLink);
                                    }
                                }
                            }
                        }
                    }
                }
            }
            iSwApp.ActiveDoc.Close();
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


            jointEstimation estimation = new jointEstimation(iSwApp);
            joint Joint = estimation.estimateJointFromComponents((AssemblyDoc)ActiveSWModel, parent.SWComponent, child.SWComponent, true);
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
            string filename = exportFiles(mRobot.BaseLink, package);
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
            string meshFileName = package.MeshesDirectory + Link.name + ".STL";
            string windowsMeshFileName = package.WindowsMeshesDirectory + Link.name + ".STL";

            int errors = 0;
            int warnings = 0;

            if (!System.IO.File.Exists(windowsMeshFileName))
            {
                ModelDoc2 modeldoc = Link.SWComponent.GetModelDoc2();

                iSwApp.ActivateDoc3(Link.name + ".sldprt", false, (int)swRebuildOnActivation_e.swUserDecision, ref errors);
                modeldoc = iSwApp.IActiveDoc2;
                int saveOptions = (int)swSaveAsOptions_e.swSaveAsOptions_Silent;
                modeldoc.Extension.SaveAs(windowsMeshFileName, (int)swSaveAsVersion_e.swSaveAsCurrentVersion, saveOptions, null, ref errors, ref warnings);
                iSwApp.CloseDoc(Link.name + ".sldprt");

                correctSTLMesh(windowsMeshFileName);
            }
            return meshFileName;
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