/*
Copyright (c) 2015 Stephen Brawner

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

using log4net;
using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.swconst;
using SW2URDF.URDF;
using System.Collections.Generic;
using System.Text;
using System.Windows.Forms;

namespace SW2URDF
{
    public static class Common
    {
        private static readonly ILog logger = Logger.GetLogger();

        //Selects the components of a link. Helps highlight when the associated node is
        // selected from the tree
        public static void SelectComponents(ModelDoc2 model, Link Link, bool clearSelection, int mark = -1)
        {
            if (clearSelection)
            {
                model.ClearSelection2(true);
            }
            SelectionMgr manager = model.SelectionManager;
            SelectData data = manager.CreateSelectData();
            data.Mark = mark;
            SelectComponents(model, Link.SWComponents, false);
            foreach (Link child in Link.Children)
            {
                SelectComponents(model, child, false, mark);
            }
        }

        //Selects components from a list.
        public static void SelectComponents(
            ModelDoc2 model, List<Component2> components, bool clearSelection = true, int mark = -1)
        {
            if (clearSelection)
            {
                model.ClearSelection2(true);
            }
            SelectionMgr manager = model.SelectionManager;
            SelectData data = manager.CreateSelectData();
            data.Mark = mark;
            foreach (Component2 component in components)
            {
                component.Select4(true, data, false);
            }
        }

        //Finds the selected components and returns them, used when pulling the items from
        // the selection box because it would be too hard for SolidWorks to allow you to
        // access the selectionbox components directly.
        public static void GetSelectedComponents(
            ModelDoc2 model, List<Component2> Components, int Mark = -1)
        {
            SelectionMgr selectionManager = model.SelectionManager;
            Components.Clear();
            for (int i = 0; i < selectionManager.GetSelectedObjectCount2(Mark); i++)
            {
                object obj = selectionManager.GetSelectedObject6(i + 1, Mark);
                Component2 comp = (Component2)obj;
                if (comp != null)
                {
                    Components.Add(comp);
                }
            }
        }

        //finds all the hidden components, which will be added to a new display state. Also
        // used when exporting STLs, so that hidden components remain hidden
        public static List<string> FindHiddenComponents(object[] varComp)
        {
            List<string> hiddenComp = new List<string>();
            foreach (object obj in varComp)
            {
                Component2 comp = (Component2)obj;
                if (comp.IsHidden(false))
                {
                    hiddenComp.Add(comp.Name2);
                }
            }
            return hiddenComp;
        }

        //Except for an exclusionary list, this shows all the components
        public static void ShowAllComponents(ModelDoc2 model, List<string> hiddenComponents)
        {
            AssemblyDoc assyDoc = (AssemblyDoc)model;
            List<Component2> componentsToShow = new List<Component2>();
            object[] varComps = assyDoc.GetComponents(false);
            foreach (Component2 comp in varComps)
            {
                if (!hiddenComponents.Contains(comp.Name2))
                {
                    componentsToShow.Add(comp);
                }
            }
            ShowComponents(model, componentsToShow);
        }

        //Shows the components in the list. Useful  for exporting STLs
        public static void ShowComponents(ModelDoc2 model, List<Component2> components)
        {
            SelectComponents(model, components, true);
            model.ShowComponent2();
        }

        //Shows the components in the link
        public static void ShowComponents(ModelDoc2 model, Link Link)
        {
            SelectComponents(model, Link, true);
            model.ShowComponent2();
        }

        //Hides the components from a link
        public static void HideComponents(ModelDoc2 model, Link Link)
        {
            SelectComponents(model, Link, true);
            model.HideComponent2();
        }

        //Hides the components from a list
        public static void HideComponents(ModelDoc2 model, List<Component2> components)
        {
            SelectComponents(model, components, true);
            model.HideComponent2();
        }

        public static int GetCount(Link Link)
        {
            int count = 1;
            foreach (Link child in Link.Children)
            {
                count += GetCount(child);
            }
            return count;
        }

        public static int GetCount(LinkNode node)
        {
            int count = 1;
            foreach (LinkNode child in node.Nodes)
            {
                count += GetCount(child);
            }
            return count;
        }

        public static int GetCount(TreeNodeCollection nodes)
        {
            int count = 0;
            foreach (LinkNode node in nodes)
            {
                count += GetCount(node);
            }
            return count;
        }

        public static void RetrieveSWComponentPIDs(ModelDoc2 model, LinkNode node)
        {
            if (node.Link.SWComponents != null)
            {
                node.Link.SWComponentPIDs = new List<byte[]>();
                foreach (IComponent2 comp in node.Link.SWComponents)
                {
                    byte[] PID = model.Extension.GetPersistReference3(comp);
                    node.Link.SWComponentPIDs.Add(PID);
                }
            }
            foreach (LinkNode child in node.Nodes)
            {
                RetrieveSWComponentPIDs(model, child);
            }
        }

        public static void RetrieveSWComponentPIDs(ModelDoc2 model, TreeView tree)
        {
            foreach (LinkNode node in tree.Nodes)
            {
                RetrieveSWComponentPIDs(model, node);
            }
        }

        //Converts the SW component references to PIDs
        public static void SaveSWComponents(ModelDoc2 model, Link Link)
        {
            model.ClearSelection2(true);
            byte[] PID = SaveSWComponent(model, Link.SWMainComponent);
            if (PID != null)
            {
                Link.SWMainComponentPID = PID;
            }
            Link.SWComponentPIDs = SaveSWComponents(model, Link.SWComponents);

            foreach (Link Child in Link.Children)
            {
                SaveSWComponents(model, Child);
            }
        }

        //Converts SW component references to PIDs
        public static List<byte[]> SaveSWComponents(ModelDoc2 model, List<Component2> components)
        {
            List<byte[]> PIDs = new List<byte[]>();
            foreach (Component2 component in components)
            {
                byte[] PID = SaveSWComponent(model, component);
                if (PID != null)
                {
                    PIDs.Add(PID);
                }
            }
            return PIDs;
        }

        public static byte[] SaveSWComponent(ModelDoc2 model, Component2 component)
        {
            if (component != null)
            {
                return model.Extension.GetPersistReference3(component);
            }
            return null;
        }

        // Converts the PIDs to actual references to the components and proceeds recursively
        // through the child nodes
        public static void LoadSWComponents(ModelDoc2 model, LinkNode node, List<string> problemLinks)
        {
            logger.Info("Loading SolidWorks components for " +
                node.Link.Name + " from " + model.GetPathName());

            node.Link.SWComponents = LoadSWComponents(model, node.Link.SWComponentPIDs);
            if (node.Link.SWComponents.Count != node.Link.SWComponentPIDs.Count)
            {
                problemLinks.Add(node.Link.Name);
                logger.Error("Link " + node.Link.Name + " did not fully load all components");
            }
            logger.Info("Loaded " + node.Link.SWComponents.Count + " components for link " + node.Link.Name);

            foreach (LinkNode Child in node.Nodes)
            {
                LoadSWComponents(model, Child, problemLinks);
            }
        }

        // Converts the PIDs to actual references to the components
        public static List<Component2> LoadSWComponents(ModelDoc2 model, List<byte[]> PIDs)
        {
            List<Component2> components = new List<Component2>();
            foreach (byte[] PID in PIDs)
            {
                string byteAsString = PIDToString(PID);
                logger.Info("Loading component with PID " + byteAsString);
                Component2 comp = LoadSWComponent(model, PID);
                if (comp == null)
                {
                    logger.Warn("Component with PID " + byteAsString + " failed to load");
                }
                else
                {
                    components.Add(comp);
                    logger.Info("Successfully loaded component " + comp.GetPathName());
                }
            }
            return components;
        }

        // Converts a single PID to a Component2 object
        public static Component2 LoadSWComponent(ModelDoc2 model, byte[] PID)
        {
            int Errors = 0;
            string byteAsString = PIDToString(PID);
            if (PID != null)
            {
                object obj = model.Extension.GetObjectByPersistReference3(PID, out Errors);
                Component2 component = (Component2)obj;
                return component;
            }
            else
            {
                logger.Error("PID " + byteAsString + " was null. Is the configuration corrupted?");
            }
            if (Errors != 0)
            {
                switch ((swPersistReferencedObjectStates_e)Errors)
                {
                    case swPersistReferencedObjectStates_e.swPersistReferencedObject_Deleted:
                        logger.Error("The component associated with PID " + byteAsString + " was deleted");
                        break;

                    case swPersistReferencedObjectStates_e.swPersistReferencedObject_Invalid:
                        logger.Error("The component associated with PID " + byteAsString + " was found to be invalid");
                        break;

                    case swPersistReferencedObjectStates_e.swPersistReferencedObject_Suppressed:
                        logger.Error("The component associated with PID " + byteAsString + " is suppressed");
                        break;

                    case swPersistReferencedObjectStates_e.swPersistReferencedObject_Ok:
                        break;

                    default:
                        logger.Error("The component associated with PID " + byteAsString +
                            " was not loaded due to an unspecified error (" + Errors + ")");
                        break;
                }
            }
            return null;
        }

        public static string PIDToString(byte[] pid)
        {
            return Encoding.ASCII.GetString(pid);
        }
    }
}