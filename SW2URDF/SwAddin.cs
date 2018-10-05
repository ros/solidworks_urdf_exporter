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

using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.swconst;
using SolidWorks.Interop.swpublished;
using SolidWorksTools;
using System;
using System.Collections;
using System.Runtime.InteropServices;
using System.Threading;
using System.Windows.Forms;

namespace SW2URDF
{
    // Adding a new line
    //
    /// <summary>
    /// Summary description for SW2URDF.
    /// </summary>
    [Guid("65c9fc17-6a74-45a3-8f84-55185900275d"), ComVisible(true)]
    [SwAddin(
        Description = "SolidWorks to URDF exporter",
        Title = "SW2URDF",
        LoadAtStartup = true
        )]
    public class SwAddin : ISwAddin
    {
        #region Static Variables

        private static readonly log4net.ILog logger = Logger.GetLogger();

        #endregion Static Variables

        #region Local Variables

        private int addinID = 0;

        public const int mainCmdGroupID = 5;
        public const int mainItemID1 = 0;
        public const int mainItemID2 = 1;
        public const int mainItemID3 = 2;
        public const int flyoutGroupID = 91;

        #region Event Handler Variables

        private SldWorks SwEventPtr = null;

        #endregion Event Handler Variables

        // Public Properties
        public ISldWorks SwApp { get; private set; } = null;

        public ICommandManager CmdMgr { get; private set; } = null;

        public Hashtable OpenDocs { get; private set; } = new Hashtable();

        #endregion Local Variables

        #region SolidWorks Registration

        [ComRegisterFunction]
        public static void RegisterFunction(Type t)
        {
            #region Get Custom Attribute: SwAddinAttribute

            SwAddinAttribute SWattr = null;
            Type type = typeof(SwAddin);

            foreach (System.Attribute attr in type.GetCustomAttributes(false))
            {
                if (attr is SwAddinAttribute)
                {
                    SWattr = attr as SwAddinAttribute;
                    break;
                }
            }

            #endregion Get Custom Attribute: SwAddinAttribute

            try
            {
                Microsoft.Win32.RegistryKey hklm = Microsoft.Win32.Registry.LocalMachine;
                Microsoft.Win32.RegistryKey hkcu = Microsoft.Win32.Registry.CurrentUser;

                string keyname = "SOFTWARE\\SolidWorks\\Addins\\{" + t.GUID.ToString() + "}";
                logger.Info("Registering " + keyname);
                Microsoft.Win32.RegistryKey addinkey = hklm.CreateSubKey(keyname);
                addinkey.SetValue(null, 0);

                addinkey.SetValue("Description", SWattr.Description);
                addinkey.SetValue("Title", SWattr.Title);

                keyname = "Software\\SolidWorks\\AddInsStartup\\{" + t.GUID.ToString() + "}";
                logger.Info("Registering " + keyname);
                addinkey = hkcu.CreateSubKey(keyname);
                addinkey.SetValue(
                    null, Convert.ToInt32(SWattr.LoadAtStartup), Microsoft.Win32.RegistryValueKind.DWord);
            }
            catch (NullReferenceException nl)
            {
                logger.Error("There was a problem registering this dll: SWattr is null. \n\"" +
                    nl.Message + "\"", nl);
                MessageBox.Show("There was a problem registering this dll: SWattr is null. \n\"" +
                    nl.Message + "\"\nEmail your maintainer with the log file found at " + Logger.GetFileName());
            }
            catch (Exception e)
            {
                logger.Error(e.Message);
                MessageBox.Show("There was a problem registering the function: \n\"" + e.Message +
                    "\"\nEmail your maintainer with the log file found at " + Logger.GetFileName());
            }
        }

        [ComUnregisterFunction]
        public static void UnregisterFunction(Type t)
        {
            try
            {
                Microsoft.Win32.RegistryKey hklm = Microsoft.Win32.Registry.LocalMachine;
                Microsoft.Win32.RegistryKey hkcu = Microsoft.Win32.Registry.CurrentUser;

                string keyname = "SOFTWARE\\SolidWorks\\Addins\\{" + t.GUID.ToString() + "}";
                logger.Info("Unregistering " + keyname);
                hklm.DeleteSubKey(keyname);

                keyname = "Software\\SolidWorks\\AddInsStartup\\{" + t.GUID.ToString() + "}";
                logger.Info("Unregistering " + keyname);
                hkcu.DeleteSubKey(keyname);
            }
            catch (NullReferenceException nl)
            {
                logger.Error("There was a problem unregistering this dll: " + nl.Message);
                MessageBox.Show("There was a problem unregistering this dll: \n\"" +
                    nl.Message + "\"\nEmail your maintainer with the log file found at " +
                    Logger.GetFileName());
            }
            catch (Exception e)
            {
                logger.Error("There was a problem unregistering this dll: " + e.Message);
                MessageBox.Show("There was a problem unregistering this dll: \n\"" +
                    e.Message + "\"\nEmail your maintainer with the log file found at " +
                    Logger.GetFileName());
            }
        }

        #endregion SolidWorks Registration

        #region ISwAddin Implementation

        public SwAddin()
        {
            Application.ThreadException +=
                new ThreadExceptionEventHandler(ExceptionHandler);
            Application.SetUnhandledExceptionMode(UnhandledExceptionMode.CatchException);
            AppDomain.CurrentDomain.UnhandledException +=
                new UnhandledExceptionEventHandler(UnhandledException);
            Logger.Setup();
        }

        private void ExceptionHandler(object sender, ThreadExceptionEventArgs e)
        {
            logger.Warn("Exception encountered in Assembly export form", e.Exception);
        }

        private void UnhandledException(object sender, UnhandledExceptionEventArgs e)
        {
            logger.Error("Unhandled exception in Assembly Export form\nEmail your maintainer " +
                "with the log file found at " +
                Logger.GetFileName(), (Exception)e.ExceptionObject);
        }

        public bool ConnectToSW(object ThisSW, int cookie)
        {
            SwApp = (ISldWorks)ThisSW;
            addinID = cookie;

            //Setup callbacks
            SwApp.SetAddinCallbackInfo(0, this, addinID);

            #region Setup the Command Manager

            CmdMgr = SwApp.GetCommandManager(cookie);
            AddCommandMgr();

            #endregion Setup the Command Manager

            #region Setup the Event Handlers

            SwEventPtr = (SldWorks)SwApp;
            OpenDocs = new Hashtable();
            AttachEventHandlers();

            #endregion Setup the Event Handlers

            logger.Info("Connecting plugin to SolidWorks");
            return true;
        }

        public bool DisconnectFromSW()
        {
            RemoveCommandMgr();
            DetachEventHandlers();

            Marshal.ReleaseComObject(CmdMgr);
            CmdMgr = null;
            Marshal.ReleaseComObject(SwApp);
            SwApp = null;
            //The addin _must_ call GC.Collect() here in order to retrieve all managed code pointers
            GC.Collect();
            GC.WaitForPendingFinalizers();

            GC.Collect();
            GC.WaitForPendingFinalizers();

            logger.Info("Disconnecting plugin from SolidWorks");
            return true;
        }

        #endregion ISwAddin Implementation

        #region UI Methods

        public void AddCommandMgr()
        {
            SwApp.AddMenuItem3((int)swDocumentTypes_e.swDocASSEMBLY, addinID, "Export as URDF@&File",
                10, "AssemblyURDFExporter", "", "Export assembly as URDF file", "");
            logger.Info("Adding Assembly export to file menu");
            SwApp.AddMenuItem3((int)swDocumentTypes_e.swDocPART, addinID, "Export as URDF@&File",
                10, "PartURDFExporter", "", "Export part as URDF file", "");
            logger.Info("Adding Part export to file menu");
        }

        public void RemoveCommandMgr()
        {
            SwApp.RemoveMenu((int)swDocumentTypes_e.swDocASSEMBLY, "Export as URDF@&File", "");
            logger.Info("Removing assembly export from file menu");
            SwApp.RemoveMenu((int)swDocumentTypes_e.swDocPART, "Export as URDF@&File", "");
            logger.Info("Removing part export from file menu");
        }

        #endregion UI Methods

        #region UI Callbacks

        public void SetupAssemblyExporter()
        {
            ModelDoc2 modeldoc = SwApp.ActiveDoc;
            logger.Info("Assembly export called for file " + modeldoc.GetTitle());
            bool saveAndRebuild = false;
            if (modeldoc.GetSaveFlag())
            {
                saveAndRebuild = true;
                logger.Info("Save is required");
            }
            else if (modeldoc.Extension.NeedsRebuild2 !=
                (int)swModelRebuildStatus_e.swModelRebuildStatus_FullyRebuilt)
            {
                saveAndRebuild = true;
                logger.Info("A rebuild is required");
            }
            if (saveAndRebuild ||
                MessageBox.Show("The SW to URDF exporter requires saving and/or rebuilding before continuing",
                "Save and rebuild document?", MessageBoxButtons.YesNo) == DialogResult.Yes)
            {
                int options = (int)swSaveAsOptions_e.swSaveAsOptions_SaveReferenced |
                        (int)swSaveAsOptions_e.swSaveAsOptions_Silent;
                logger.Info("Saving assembly");
                modeldoc.Save3(options, 0, 0);

                logger.Info("Opening property manager");
                SetupPropertyManager();
            }
        }

        public void AssemblyURDFExporter()
        {
            try
            {
                SetupAssemblyExporter();
            }
            catch (Exception e)
            {
                logger.Error("An exception was caught when trying to setup the assembly exporter", e);
                MessageBox.Show("There was a problem setting up the property manager: \n\"" +
                    e.Message + "\"\nEmail your maintainer with the log file found at " +
                    Logger.GetFileName());
            }
        }

        public void SetupPropertyManager()
        {
            URDFExporterPM pm = new URDFExporterPM((SldWorks)SwApp);
            logger.Info("Loading config tree");
            bool success = pm.LoadConfigTree();

            if (success)
            {
                logger.Info("Showing property manager");
                pm.Show();
            }
        }

        public void SetupPartExporter()
        {
            logger.Info("Part export called");
            ModelDoc2 modeldoc = SwApp.ActiveDoc;
            if ((modeldoc.Extension.NeedsRebuild2 == 0) ||
                MessageBox.Show("Save and rebuild document?",
                "The SW to URDF exporter requires saving before continuing",
                MessageBoxButtons.YesNo) == DialogResult.Yes)
            {
                if (modeldoc.Extension.NeedsRebuild2 != 0)
                {
                    int options = (int)swSaveAsOptions_e.swSaveAsOptions_SaveReferenced |
                        (int)swSaveAsOptions_e.swSaveAsOptions_Silent;
                    logger.Info("Saving part");
                    modeldoc.Save3(options, 0, 0);
                }

                PartExportForm exportForm = new PartExportForm((SldWorks)SwApp);
                logger.Info("Showing part");
                exportForm.Show();
            }
        }

        public void PartURDFExporter()
        {
            try
            {
                SetupPartExporter();
            }
            catch (Exception e)
            {
                logger.Error("Excoption caught setting up export form", e);
                MessageBox.Show("An exception occured setting up the export form, please email " +
                    " your maintainer with the log file found at " + Logger.GetFileName());
            }
        }

        public void FlyoutCallback()
        {
            FlyoutGroup flyGroup = CmdMgr.GetFlyoutGroup(flyoutGroupID);
            flyGroup.RemoveAllCommandItems();

            flyGroup.AddCommandItem(
                DateTime.Now.ToLongTimeString(), "test", 0, "FlyoutCommandItem1", "FlyoutEnableCommandItem1");
        }

        public int FlyoutEnable()
        {
            return 1;
        }

        public void FlyoutCommandItem1()
        {
            SwApp.SendMsgToUser("Flyout command 1");
        }

        public int FlyoutEnableCommandItem1()
        {
            return 1;
        }

        #endregion UI Callbacks

        #region Event Methods

        public bool AttachEventHandlers()
        {
            AttachSwEvents();
            //Listen for events on all currently open docs
            AttachEventsToAllDocuments();
            return true;
        }

        private bool AttachSwEvents()
        {
            try
            {
                SwEventPtr.ActiveDocChangeNotify +=
                    new DSldWorksEvents_ActiveDocChangeNotifyEventHandler(OnDocChange);
                SwEventPtr.DocumentLoadNotify2 +=
                    new DSldWorksEvents_DocumentLoadNotify2EventHandler(OnDocLoad);
                SwEventPtr.FileNewNotify2 +=
                    new DSldWorksEvents_FileNewNotify2EventHandler(OnFileNew);
                SwEventPtr.ActiveModelDocChangeNotify +=
                    new DSldWorksEvents_ActiveModelDocChangeNotifyEventHandler(OnModelChange);
                SwEventPtr.FileOpenPostNotify +=
                    new DSldWorksEvents_FileOpenPostNotifyEventHandler(FileOpenPostNotify);
                return true;
            }
            catch (Exception e)
            {
                logger.Error("Attaching SW events failed", e);
                return false;
            }
        }

        private bool DetachSwEvents()
        {
            try
            {
                SwEventPtr.ActiveDocChangeNotify -=
                    new DSldWorksEvents_ActiveDocChangeNotifyEventHandler(OnDocChange);
                SwEventPtr.DocumentLoadNotify2 -=
                    new DSldWorksEvents_DocumentLoadNotify2EventHandler(OnDocLoad);
                SwEventPtr.FileNewNotify2 -=
                    new DSldWorksEvents_FileNewNotify2EventHandler(OnFileNew);
                SwEventPtr.ActiveModelDocChangeNotify -=
                    new DSldWorksEvents_ActiveModelDocChangeNotifyEventHandler(OnModelChange);
                SwEventPtr.FileOpenPostNotify -=
                    new DSldWorksEvents_FileOpenPostNotifyEventHandler(FileOpenPostNotify);
                return true;
            }
            catch (Exception e)
            {
                logger.Error("Attaching SW events failed", e);
                return false;
            }
        }

        public void AttachEventsToAllDocuments()
        {
            ModelDoc2 modDoc = (ModelDoc2)SwApp.GetFirstDocument();
            while (modDoc != null)
            {
                if (!OpenDocs.Contains(modDoc))
                {
                    AttachModelDocEventHandler(modDoc);
                }
                else if (OpenDocs.Contains(modDoc))
                {
                    bool connected = false;
                    DocumentEventHandler docHandler = (DocumentEventHandler)OpenDocs[modDoc];
                    if (docHandler != null)
                    {
                        connected = docHandler.ConnectModelViews();
                    }
                }

                modDoc = (ModelDoc2)modDoc.GetNext();
            }
        }

        public bool AttachModelDocEventHandler(ModelDoc2 modDoc)
        {
            if (modDoc == null)
            {
                return false;
            }

            DocumentEventHandler docHandler = null;

            if (!OpenDocs.Contains(modDoc))
            {
                switch (modDoc.GetType())
                {
                    case (int)swDocumentTypes_e.swDocPART:
                        {
                            docHandler = new PartEventHandler(modDoc, this);
                            break;
                        }
                    case (int)swDocumentTypes_e.swDocASSEMBLY:
                        {
                            docHandler = new AssemblyEventHandler(modDoc, this);
                            break;
                        }
                    case (int)swDocumentTypes_e.swDocDRAWING:
                        {
                            docHandler = new DrawingEventHandler(modDoc, this);
                            break;
                        }
                    default:
                        {
                            return false; //Unsupported document type
                        }
                }
                docHandler.AttachEventHandlers();
                OpenDocs.Add(modDoc, docHandler);
            }
            return true;
        }

        public bool DetachModelEventHandler(ModelDoc2 modDoc)
        {
            DocumentEventHandler docHandler;
            docHandler = (DocumentEventHandler)OpenDocs[modDoc];
            OpenDocs.Remove(modDoc);
            modDoc = null;
            docHandler = null;
            return true;
        }

        public bool DetachEventHandlers()
        {
            DetachSwEvents();

            //Close events on all currently open docs
            DocumentEventHandler docHandler;
            int numKeys = OpenDocs.Count;
            object[] keys = new Object[numKeys];

            //Remove all document event handlers
            OpenDocs.Keys.CopyTo(keys, 0);
            foreach (ModelDoc2 key in keys)
            {
                docHandler = (DocumentEventHandler)OpenDocs[key];
                docHandler.DetachEventHandlers(); //This also removes the pair from the hash
                docHandler = null;
            }
            return true;
        }

        #endregion Event Methods

        #region Event Handlers

        //Events
        public int OnDocChange()
        {
            return 0;
        }

        public int OnDocLoad(string docTitle, string docPath)
        {
            return 0;
        }

        private int FileOpenPostNotify(string FileName)
        {
            AttachEventsToAllDocuments();
            return 0;
        }

        public int OnFileNew(object newDoc, int docType, string templateName)
        {
            AttachEventsToAllDocuments();
            return 0;
        }

        public int OnModelChange()
        {
            return 0;
        }

        #endregion Event Handlers
    }
}
