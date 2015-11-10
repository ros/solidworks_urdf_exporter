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

using System.Collections.Generic;
using System.Windows.Forms;
using SolidWorks.Interop.sldworks;

namespace SW2URDF
{
    public static class Common
    {
        //Selects the components of a link. Helps highlight when the associated node is selected from the tree
        public static void selectComponents(ModelDoc2 model, link Link, bool clearSelection, int mark = -1)
        {
            if (clearSelection)
            {
                model.ClearSelection2(true);
            }
            SelectionMgr manager = model.SelectionManager;
            SelectData data = manager.CreateSelectData();
            data.Mark = mark;
            if (Link.SWComponent != null)
            {
                Link.SWComponent.Select4(true, data, false);
            }
            else
            {
                selectComponents(model, Link.SWcomponents, false);
            }
            foreach (link child in Link.Children)
            {
                selectComponents(model, child, false, mark);
            }
        }

        //Selects components from a list.
        public static void selectComponents(ModelDoc2 model, List<Component2> components, bool clearSelection = true, int mark = -1)
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

        //Finds the selected components and returns them, used when pulling the items from the selection box because it would be too hard
        // for SolidWorks to allow you to access the selectionbox components directly.
        public static void getSelectedComponents(ModelDoc2 model, List<Component2> Components, int Mark = -1)
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


        //finds all the hidden components, which will be added to a new display state. Also used when exporting STLs, so that hidden components
        //remain hidden
        public static List<string> findHiddenComponents(object[] varComp)
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
        public static void showAllComponents(ModelDoc2 model, List<string> hiddenComponents)
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
            showComponents(model, componentsToShow);
        }

        //Shows the components in the list. Useful  for exporting STLs
        public static void showComponents(ModelDoc2 model, List<Component2> components)
        {
            selectComponents(model, components, true);
            model.ShowComponent2();
        }

        //Shows the components in the link
        public static void showComponents(ModelDoc2 model, link Link)
        {
            selectComponents(model, Link, true);
            model.ShowComponent2();

        }

        //Hides the components from a link
        public static void hideComponents(ModelDoc2 model, link Link)
        {
            selectComponents(model, Link, true);
            model.HideComponent2();
        }

        //Hides the components from a list
        public static void hideComponents(ModelDoc2 model, List<Component2> components)
        {
            selectComponents(model, components, true);
            model.HideComponent2();
        }

        public static int getCount(link Link)
        {
            int count = 1;
            foreach (link child in Link.Children)
            {
                count += getCount(child);
            }
            return count;
        }

        public static int getCount(LinkNode node)
        {
            int count = 1;
            foreach (LinkNode child in node.Nodes)
            {
                count += getCount(child);
            }
            return count;
        }

        public static int getCount(TreeNodeCollection nodes)
        {
            int count = 0;
            foreach (LinkNode node in nodes)
            {
                count += getCount(node);
            }
            return count;
        }

        public static void retrieveSWComponentPIDs(ModelDoc2 model, LinkNode node)
        {
            if (node.Components != null)
            {
                node.ComponentPIDs = new List<byte[]>();
                foreach (IComponent2 comp in node.Components)
                {
                    byte[] PID = model.Extension.GetPersistReference3(comp);
                    node.ComponentPIDs.Add(PID);
                }
            }
            foreach (LinkNode child in node.Nodes)
            {
                retrieveSWComponentPIDs(model, child);
            }
        }

        public static void retrieveSWComponentPIDs(ModelDoc2 model, TreeView tree)
        {
            foreach (LinkNode node in tree.Nodes)
            {
                retrieveSWComponentPIDs(model, node);
            }
        }



        //Converts the SW component references to PIDs
        public static void saveSWComponents(ModelDoc2 model, link Link)
        {
            model.ClearSelection2(true);
            byte[] PID = saveSWComponent(model, Link.SWMainComponent);
            if (PID != null)
            {
                Link.SWMainComponentPID = PID;
            }
            Link.SWComponentPIDs = saveSWComponents(model, Link.SWcomponents);

            foreach (link Child in Link.Children)
            {
                saveSWComponents(model, Child);
            }
        }

        //Converts SW component references to PIDs
        public static List<byte[]> saveSWComponents(ModelDoc2 model, List<Component2> components)
        {
            List<byte[]> PIDs = new List<byte[]>();
            foreach (Component2 component in components)
            {
                byte[] PID = saveSWComponent(model, component);
                if (PID != null)
                {
                    PIDs.Add(PID);
                }
            }
            return PIDs;
        }

        public static byte[] saveSWComponent(ModelDoc2 model, Component2 component)
        {
            if (component != null)
            {
                return model.Extension.GetPersistReference3(component);
            }
            return null;
        }


        // Converts the PIDs to actual references to the components and proceeds recursively through the child links
        public static void loadSWComponents(ModelDoc2 model, link Link)
        {
            Link.SWMainComponent = loadSWComponent(model, Link.SWMainComponentPID);
            Link.SWcomponents = loadSWComponents(model, Link.SWComponentPIDs);
            foreach (link Child in Link.Children)
            {
                loadSWComponents(model, Child);
            }
        }

        // Converts the PIDs to actual references to the components and proceeds recursively through the child nodes
        public static void loadSWComponents(ModelDoc2 model, LinkNode node)
        {
            node.Components = loadSWComponents(model, node.ComponentPIDs);

            foreach (LinkNode Child in node.Nodes)
            {
                loadSWComponents(model, Child);
            }
        }

        // Converts the PIDs to actual references to the components
        public static List<Component2> loadSWComponents(ModelDoc2 model, List<byte[]> PIDs)
        {
            List<Component2> components = new List<Component2>();
            foreach (byte[] PID in PIDs)
            {
                components.Add(loadSWComponent(model, PID));
            }
            return components;
        }

        // Converts a single PID to a Component2 object
        public static Component2 loadSWComponent(ModelDoc2 model, byte[] PID)
        {
            int Errors = 0;
            if (PID != null)
            {
                return (Component2)model.Extension.GetObjectByPersistReference3(PID, out Errors);
            }
            return null;
        }
    }
}
