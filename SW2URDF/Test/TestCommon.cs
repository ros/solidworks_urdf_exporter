using SolidWorks.Interop.sldworks;
using SW2URDF.URDF;
using SW2URDF.URDFExport;
using System.Collections.Generic;
using System.Linq;
using Xunit;

namespace SW2URDF.Test
{
    [Collection("Requires SW Test Collection")]
    public class TestCommon : SW2URDFTest
    {
        public TestCommon(SWTestFixture fixture) : base(fixture)
        {

        }

        private static void AddLinkComponents(Link link, List<Component2> components)
        {
            components.AddRange(link.SWComponents);
            foreach (Link child in link.Children)
            {
                AddLinkComponents(child, components);
            }
        }

        /// <summary>
        /// Tests selecting the components of a link.
        /// </summary>
        /// <param name="modelName"></param>
        [Theory]
        [InlineData(MODEL_NAME_3_DOF_ARM)]
        public void TestSelectComponentsLink(string modelName)
        {
            ModelDoc2 doc = OpenSWDocument(modelName);
            AssemblyDoc assyDoc = (AssemblyDoc)doc;
            LinkNode baseNode = Serialization.LoadBaseNodeFromModel(doc, out bool abortProcess);
            Assert.False(abortProcess);

            Link baseLink = baseNode.GetLink();
            List<Component2> componentsToSelect = new List<Component2>();
            AddLinkComponents(baseLink, componentsToSelect);
            HashSet<string> componentsToSelectNames = 
                new HashSet<string>(componentsToSelect.Select(component => component.Name2));

            Common.SelectComponents(doc, baseLink, true);
            SelectionMgr selManager = doc.SelectionManager;
            int numSelected = selManager.GetSelectedObjectCount2(-1);
            Assert.Equal(componentsToSelect.Count, numSelected);

            for (int i = 0; i < selManager.GetSelectedObjectCount2(-1); i++)
            {
                Component2 comp = selManager.GetSelectedObjectsComponent4(i, -1);
                Assert.Contains(comp.Name2, componentsToSelectNames);
            }

            SwApp.CloseAllDocuments(true);
        }

        /// <summary>
        /// Tests selecting components from a Component2 list.
        /// </summary>
        /// <param name="modelName"></param>
        [Theory]
        [InlineData(MODEL_NAME_3_DOF_ARM)]
        public void TestSelectComponentsList(string modelName)
        {
            ModelDoc2 doc = OpenSWDocument(modelName);
            AssemblyDoc assyDoc = (AssemblyDoc)doc;
            LinkNode baseNode = Serialization.LoadBaseNodeFromModel(doc, out bool abortProcess);
            Assert.False(abortProcess);

            Link baseLink = baseNode.GetLink();
            List<Component2> componentsToSelect = baseLink.SWComponents;
            HashSet<string> componentsToSelectNames = 
                new HashSet<string>(componentsToSelect.Select(component => component.Name2));

            Common.SelectComponents(doc, componentsToSelect, true);
            SelectionMgr selManager = doc.SelectionManager;

            // -1 is the Mark, set to negative one if it's not being used.
            int numSelected = selManager.GetSelectedObjectCount2(-1);
            Assert.Equal(componentsToSelect.Count, numSelected);

            for (int i = 0; i < selManager.GetSelectedObjectCount2(-1); i++)
            {
                // -1 is the Mark, set to negative one if it's not being used.
                Component2 comp = selManager.GetSelectedObjectsComponent4(i, -1);
                Assert.Contains(comp.Name2, componentsToSelectNames);
            }

            SwApp.CloseAllDocuments(true);
        }

        /// <summary>
        /// Test that the selected components in a model are properly retrieved.
        /// </summary>
        /// <param name="modelName"></param>
        [Theory]
        [InlineData(MODEL_NAME_3_DOF_ARM)]
        public void TestGetSelectedComponents(string modelName)
        {
            ModelDoc2 doc = OpenSWDocument(modelName);
            AssemblyDoc assyDoc = (AssemblyDoc)doc;
            LinkNode baseNode = Serialization.LoadBaseNodeFromModel(doc, out bool abortProcess);
            Assert.False(abortProcess);

            Link baseLink = baseNode.GetLink();
            List<Component2> componentsToSelect = baseLink.SWComponents;
            HashSet<string> componentsToSelectNames = 
                new HashSet<string>(componentsToSelect.Select(component => component.Name2));

            Common.SelectComponents(doc, componentsToSelect, true);
            List<Component2> selectedComponents = new List<Component2>();
            Common.GetSelectedComponents(doc, selectedComponents);
            Assert.Equal(componentsToSelect.Count, selectedComponents.Count);

            SwApp.CloseAllDocuments(true);
        }

        [Theory]
        [InlineData(MODEL_NAME_3_DOF_ARM, 0)]
        [InlineData(MODEL_NAME_ORIGINAL_3_DOF_ARM, 3)]
        public void TestFindHiddenComponens(string modelName, int expected)
        {
            ModelDoc2 doc = OpenSWDocument(modelName);
            AssemblyDoc assyDoc = (AssemblyDoc)doc;
            Assert.Equal(expected, Common.FindHiddenComponents(assyDoc.GetComponents(false)).Count);

            SwApp.CloseAllDocuments(true);
        }

        [Theory]
        [InlineData(MODEL_NAME_3_DOF_ARM)]
        [InlineData(MODEL_NAME_ORIGINAL_3_DOF_ARM)]
        public void TestShowAllComponents(string modelName)
        {
            ModelDoc2 doc = OpenSWDocument(modelName);
            AssemblyDoc assyDoc = (AssemblyDoc)doc;
            Common.ShowAllComponents(doc, new List<string>());
            Assert.Equal(0, Common.FindHiddenComponents(assyDoc.GetComponents(false)).Count);

            SwApp.CloseAllDocuments(true);
        }

        [Theory]
        [InlineData(MODEL_NAME_3_DOF_ARM)]
        [InlineData(MODEL_NAME_ORIGINAL_3_DOF_ARM)]
        public void TestShowComponents(string modelName)
        {
            ModelDoc2 doc = OpenSWDocument(modelName);
            AssemblyDoc assyDoc = (AssemblyDoc)doc;

            // AssemblyDoc.GetComponentsByName only works on top level components.
            List<string> hiddenComponentNames = Common.FindHiddenComponents(assyDoc.GetComponents(true));
            List<Component2> hiddenComponents = new List<Component2>();
            foreach(string name in hiddenComponentNames)
            {
                Component2 hiddenComp = assyDoc.GetComponentByName(name);
                Assert.NotNull(hiddenComp);
                hiddenComponents.Add(hiddenComp);
            }
            Common.ShowComponents(doc, hiddenComponents);
            Assert.Equal(0, Common.FindHiddenComponents(assyDoc.GetComponents(true)).Count);

            SwApp.CloseAllDocuments(true);
        }

        [Theory]
        [InlineData(MODEL_NAME_3_DOF_ARM)]
        public void TestHideComponents(string modelName)
        {
            ModelDoc2 doc = OpenSWDocument(modelName);
            AssemblyDoc assyDoc = (AssemblyDoc)doc;
            List<string> hiddenComponentNames = Common.FindHiddenComponents(assyDoc.GetComponents(false));
            List<Component2> hiddenComponents = 
                hiddenComponentNames.Select(name => assyDoc.GetComponentByName(name)).ToList();
            Common.ShowAllComponents(doc, new List<string>());
            Common.HideComponents(doc, hiddenComponents);
            List<string> hiddenComponentNames2 = Common.FindHiddenComponents(assyDoc.GetComponents(false));
            Assert.Equal(hiddenComponentNames.Count, hiddenComponentNames2.Count);

            SwApp.CloseAllDocuments(true);
        }

        [Theory]
        [InlineData(MODEL_NAME_3_DOF_ARM, 4)]
        public void TestGetCountLink(string modelName, int expected)
        {
            ModelDoc2 doc = OpenSWDocument(modelName);
            LinkNode baseNode = Serialization.LoadBaseNodeFromModel(doc, out bool abortProcess);
            Assert.False(abortProcess);

            Link baseLink = baseNode.GetLink();
            Assert.Equal(expected, Common.GetCount(baseLink));

            SwApp.CloseAllDocuments(true);
        }

        [Theory]
        [InlineData(MODEL_NAME_3_DOF_ARM, 3)]
        public void TestGetCountNodeCollection(string modelName, int expected)
        {
            ModelDoc2 doc = OpenSWDocument(modelName);
            LinkNode baseNode = Serialization.LoadBaseNodeFromModel(doc, out bool abortProcess);
            Assert.False(abortProcess);

            Assert.Equal(expected, Common.GetCount(baseNode.Nodes));

            SwApp.CloseAllDocuments(true);
        }

        [Theory]
        [InlineData(MODEL_NAME_3_DOF_ARM)]
        public void TestRetrieveSWComponentPIDs(string modelName)
        {
            ModelDoc2 doc = OpenSWDocument(modelName);
            LinkNode baseNode = Serialization.LoadBaseNodeFromModel(doc, out bool abortProcess);
            Assert.False(abortProcess);

            Common.RetrieveSWComponentPIDs(doc, baseNode);
            Assert.Equal(baseNode.Link.SWComponents.Count, baseNode.Link.SWComponentPIDs.Count);

            SwApp.CloseAllDocuments(true);
        }

        [Theory]
        [InlineData(MODEL_NAME_3_DOF_ARM)]
        public void TestSaveSWComponentsLink(string modelName)
        {
            ModelDoc2 doc = OpenSWDocument(modelName);
            LinkNode baseNode = Serialization.LoadBaseNodeFromModel(doc, out bool abortProcess);
            Assert.False(abortProcess);

            List<string> problemLinks = new List<string>();
            Common.LoadSWComponents(doc, baseNode, problemLinks);
            Assert.Empty(problemLinks);

            Link baseLink = baseNode.GetLink();
            baseLink.SWMainComponent = baseLink.SWComponents[0];
            Common.SaveSWComponents(doc, baseLink);
            Assert.Equal(baseLink.SWComponents.Count, baseLink.SWComponentPIDs.Count);
            Assert.NotNull(baseLink.SWMainComponentPID);

            SwApp.CloseAllDocuments(true);
        }

        [Theory]
        [InlineData(MODEL_NAME_3_DOF_ARM)]
        public void TestSaveSWComponentsList(string modelName)
        {
            ModelDoc2 doc = OpenSWDocument(modelName);
            AssemblyDoc assyDoc = (AssemblyDoc)doc;
            object[] componentObjs = assyDoc.GetComponents(false);
            List<Component2> components = componentObjs.Cast<Component2>().ToList();
            List<byte[]> pids = Common.SaveSWComponents(doc, components);
            Assert.Equal(pids.Count, components.Count);

            SwApp.CloseAllDocuments(true);
        }

        [Theory]
        [InlineData(MODEL_NAME_3_DOF_ARM, "3_DOF_ARM_BASE-1", new byte[] {
            200, 46, 0, 0, 5, 0, 0, 0, 255, 254, 255, 26, 51, 0, 95, 0, 68, 0, 79, 0, 70, 0, 95,
            0, 65, 0, 82, 0, 77, 0, 95, 0, 66, 0, 65, 0, 83, 0, 69, 0, 45, 0, 49, 0, 64, 0, 51,
            0, 95, 0, 68, 0, 79, 0, 70, 0, 95, 0, 65, 0, 82, 0, 77, 0, 4, 0, 0, 0, 16, 0, 0, 0,
            1, 0, 0, 0, 1, 0, 0, 0, 17, 0, 0, 0, })]
        public void TestSaveSWComponent(string modelName, string componentName, byte[] expected)
        {
            ModelDoc2 doc = OpenSWDocument(modelName);
            AssemblyDoc assyDoc = (AssemblyDoc)doc;
            Component2 component = assyDoc.GetComponentByName(componentName);
            Assert.NotNull(component);
            LinkNode baseNode = Serialization.LoadBaseNodeFromModel(doc, out bool abortProcess);
            Assert.False(abortProcess);
            baseNode.Link.SWMainComponent = component;
            byte[] pid = Common.SaveSWComponent(doc, baseNode.Link.SWMainComponent);
            Assert.NotNull(pid);
            Assert.Equal(expected.Length, pid.Length);
            SwApp.CloseAllDocuments(true);
        }

        [Theory]
        [InlineData(MODEL_NAME_3_DOF_ARM)]
        public void TestLoadSWComponentsLinkNode(string modelName)
        {
            ModelDoc2 doc = OpenSWDocument(modelName);
            LinkNode baseNode = Serialization.LoadBaseNodeFromModel(doc, out bool abortProcess);
            Assert.False(abortProcess);
            List<string> problemLinks = new List<string>();
            Common.LoadSWComponents(doc, baseNode, problemLinks);
            Assert.Empty(problemLinks);
            
            SwApp.CloseAllDocuments(true);
        }

        [Theory]
        [InlineData(MODEL_NAME_3_DOF_ARM)]
        public void TestLoadSWComponentsList(string modelName)
        {
            ModelDoc2 doc = OpenSWDocument(modelName);
            LinkNode baseNode = Serialization.LoadBaseNodeFromModel(doc, out bool abortProcess);
            Assert.False(abortProcess);
            List<Component2> components = Common.LoadSWComponents(doc, baseNode.Link.SWComponentPIDs);
            Assert.Equal(baseNode.Link.SWComponentPIDs.Count, components.Count);

            SwApp.CloseAllDocuments(true);
        }

        [Theory]
        [InlineData(MODEL_NAME_3_DOF_ARM)]
        public void TestLoadSWComponent(string modelName)
        {
            ModelDoc2 doc = OpenSWDocument(modelName);
            LinkNode baseNode = Serialization.LoadBaseNodeFromModel(doc, out bool abortProcess);
            Assert.False(abortProcess);
            baseNode.Link.SWMainComponentPID = baseNode.Link.SWComponentPIDs[0];
            Component2 component = Common.LoadSWComponent(doc, baseNode.Link.SWMainComponentPID);
            Assert.NotNull(component);

            SwApp.CloseAllDocuments(true);
        }

        [Theory]
        [InlineData(new byte[] { }, "")]
        [InlineData(new byte[] {49, 50, 51 }, "123")]
        [InlineData(new byte[] {97, 98, 99 }, "abc")]
        public void TestPIDToString(byte[] pid, string expected)
        {
            Assert.Equal(expected, Common.PIDToString(pid));
        }
    }
}
