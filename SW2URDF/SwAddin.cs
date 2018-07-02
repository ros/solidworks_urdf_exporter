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

using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Runtime.InteropServices;
using System.Threading;
using System.Windows.Forms;
using System.Xml;
using System.Xml.Serialization;
using log4net.Appender;
using SolidWorks.Interop.sldworks;
using SolidWorks.Interop.swconst;
using SolidWorks.Interop.swpublished;
using SolidWorksTools;

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
            
        #endregion
        //
        #region Local Variables
        ISldWorks iSwApp = null;
        ICommandManager iCmdMgr = null;
        int addinID = 0;

        public const int mainCmdGroupID = 5;
        public const int mainItemID1 = 0;
        public const int mainItemID2 = 1;
        public const int mainItemID3 = 2;
        public const int flyoutGroupID = 91;

        #region Event Handler Variables
        Hashtable openDocs = new Hashtable();
        SolidWorks.Interop.sldworks.SldWorks SwEventPtr = null;
        #endregion

        #region Property Manager Variables
        #endregion


        // Public Properties
        public ISldWorks SwApp
        {
            get { return iSwApp; }
        }
        public ICommandManager CmdMgr
        {
            get { return iCmdMgr; }
        }

        public Hashtable OpenDocs
        {
            get { return openDocs; }
        }

        #endregion

        #region SolidWorks Registration

        [ComRegisterFunctionAttribute]
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

            #endregion

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
                addinkey.SetValue(null, Convert.ToInt32(SWattr.LoadAtStartup), Microsoft.Win32.RegistryValueKind.DWord);
            }
            catch (System.NullReferenceException nl)
            {
                logger.Error("There was a problem registering this dll: SWattr is null. \n\"" + nl.Message + "\"", nl);
                System.Windows.Forms.MessageBox.Show("There was a problem registering this dll: SWattr is null. \n\"" + nl.Message + "\"\nEmail your maintainer with the log file found at " + Logger.GetFileName());
            }

            catch (System.Exception e)
            {
                logger.Error(e.Message);
                System.Windows.Forms.MessageBox.Show("There was a problem registering the function: \n\"" + e.Message + "\"\nEmail your maintainer with the log file found at " + Logger.GetFileName());
            }
        }

        [ComUnregisterFunctionAttribute]
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
            catch (System.NullReferenceException nl)
            {
                logger.Error("There was a problem unregistering this dll: " + nl.Message);
                System.Windows.Forms.MessageBox.Show("There was a problem unregistering this dll: \n\"" + nl.Message + "\"\nEmail your maintainer with the log file found at " + Logger.GetFileName());
            }
            catch (System.Exception e)
            {
                logger.Error("There was a problem unregistering this dll: " + e.Message);
                System.Windows.Forms.MessageBox.Show("There was a problem unregistering this dll: \n\"" + e.Message + "\"\nEmail your maintainer with the log file found at " + Logger.GetFileName());
            }
        }

        #endregion

        #region ISwAddin Implementation
        public SwAddin()
        {
            Application.ThreadException += new ThreadExceptionEventHandler(this.exceptionHandler);
            Application.SetUnhandledExceptionMode(UnhandledExceptionMode.CatchException);
            AppDomain.CurrentDomain.UnhandledException += new UnhandledExceptionEventHandler(this.unhandledException);
            Logger.Setup();
        }

        private void exceptionHandler(object sender, ThreadExceptionEventArgs e)
        {
            logger.Warn("Exception encountered in Assembly export form", e.Exception);
        }

        private void unhandledException(object sender, UnhandledExceptionEventArgs e)
        {
            logger.Error("Unhandled exception in Assembly Export form\nEmail your maintainer with the log file found at " + Logger.GetFileName(), (System.Exception)e.ExceptionObject);
        }


        public bool ConnectToSW(object ThisSW, int cookie)
        {
            iSwApp = (ISldWorks)ThisSW;
            addinID = cookie;

            //Setup callbacks
            iSwApp.SetAddinCallbackInfo(0, this, addinID);

            #region Setup the Command Manager
            iCmdMgr = iSwApp.GetCommandManager(cookie);
            AddCommandMgr();
            #endregion

            #region Setup the Event Handlers
            SwEventPtr = (SolidWorks.Interop.sldworks.SldWorks)iSwApp;
            openDocs = new Hashtable();
            AttachEventHandlers();
            #endregion
            logger.Info("Connecting plugin to SolidWorks");
            return true;
        }

        public bool DisconnectFromSW()
        {
            RemoveCommandMgr();
            DetachEventHandlers();

            System.Runtime.InteropServices.Marshal.ReleaseComObject(iCmdMgr);
            iCmdMgr = null;
            System.Runtime.InteropServices.Marshal.ReleaseComObject(iSwApp);
            iSwApp = null;
            //The addin _must_ call GC.Collect() here in order to retrieve all managed code pointers 
            GC.Collect();
            GC.WaitForPendingFinalizers();

            GC.Collect();
            GC.WaitForPendingFinalizers();

            logger.Info("Disconnecting plugin from SolidWorks");
            return true;
        }
        #endregion

        #region UI Methods
        public void AddCommandMgr() 
        {
            iSwApp.AddMenuItem3((int)swDocumentTypes_e.swDocASSEMBLY, addinID, "Export as URDF@&File", 10, "assemblyURDFExporter", "", "Export assembly as URDF file", "");
            logger.Info("Adding Assembly export to file menu");
            iSwApp.AddMenuItem3((int)swDocumentTypes_e.swDocPART, addinID, "Export as URDF@&File", 10, "partURDFExporter", "", "Export part as URDF file", "");
            logger.Info("Adding Part export to file menu");
        }

        public void RemoveCommandMgr()
        {
            iSwApp.RemoveMenu((int)swDocumentTypes_e.swDocASSEMBLY, "Export as URDF@&File", "");
            logger.Info("Removing assembly export from file menu");
            iSwApp.RemoveMenu((int)swDocumentTypes_e.swDocPART, "Export as URDF@&File", "");
            logger.Info("Removing part export from file menu");
        }

        public bool CompareIDs(int[] storedIDs, int[] addinIDs)
        {
            List<int> storedList = new List<int>(storedIDs);
            List<int> addinList = new List<int>(addinIDs);

            addinList.Sort();
            storedList.Sort();

            if (addinList.Count != storedList.Count)
            {
                return false;
            }
            else
            {

                for (int i = 0; i < addinList.Count; i++)
                {
                    if (addinList[i] != storedList[i])
                    {
                        return false;
                    }
                }
            }
            return true;
        }

        #endregion

        #region UI Callbacks


        public void assemblyURDFExporter()
        {
            ModelDoc2 modeldoc = iSwApp.ActiveDoc;
            logger.Info("Assembly export called for file " + modeldoc.GetTitle());
            bool saveAndRebuild = false;
            if (modeldoc.GetSaveFlag())
            {
                saveAndRebuild = true;
                logger.Info("Save is required");
            }
            else if (modeldoc.Extension.NeedsRebuild2 != (int)swModelRebuildStatus_e.swModelRebuildStatus_FullyRebuilt)
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
                try
                {
                    setupPropertyManager();
                }
                catch (Exception e)
                {
                    logger.Error("An exception was caught when trying setup property mananger", e);
                    System.Windows.Forms.MessageBox.Show("There was a problem setting up the property manager: \n\"" + e.Message + "\"\nEmail your maintainer with the log file found at " + Logger.GetFileName());
                }
            }
        }

        public void setupPropertyManager()
        {
            URDFExporterPM pm = new URDFExporterPM((SldWorks)iSwApp);
            logger.Info("Loading config tree");
            pm.loadConfigTree();
            logger.Info("Showing property manager");
            pm.Show();
        }
        public void partURDFExporter()
        {
            logger.Info("Part export called");
            ModelDoc2 modeldoc = iSwApp.ActiveDoc;
            if ((modeldoc.Extension.NeedsRebuild2 == 0) || 
                MessageBox.Show("Save and rebuild document?", 
                "The SW to URDF exporter requires saving before continuing", MessageBoxButtons.YesNo) == DialogResult.Yes)
            {
                if (modeldoc.Extension.NeedsRebuild2 != 0)
                {
                    int options = (int)swSaveAsOptions_e.swSaveAsOptions_SaveReferenced | 
                        (int)swSaveAsOptions_e.swSaveAsOptions_Silent;
                    logger.Info("Saving part");
                    modeldoc.Save3(options, 0, 0);
                }
                try
                {
                    PartExportForm exportForm = new PartExportForm(iSwApp);
                    logger.Info("Showing part");
                    exportForm.Show();
                }
                catch (Exception e)
                {
                    logger.Error("Excoption caught setting up export form", e);
                    MessageBox.Show("An exception occured setting up the export form, please email your maintainer with the logfile found at " + Logger.GetFileName());
                }
                
                
            }
        }

        private URDFExporter loadConfigFile()
        {
            logger.Info("Attempting to load config file");
            ModelDoc2 modeldoc = iSwApp.ActiveDoc;

            Object[] objects = modeldoc.FeatureManager.GetFeatures(true);
            logger.Info("Retrieved " + objects.Length + " features for component");
            string data="";
            foreach (Object obj in objects)
            {
                Feature feat = (Feature)obj;
                string t = feat.GetTypeName2();
                if (feat.GetTypeName2() == "Attribute")
                {
                    SolidWorks.Interop.sldworks.Attribute att = 
                        (SolidWorks.Interop.sldworks.Attribute)feat.GetSpecificFeature2();
                    if (att.GetName() == "URDF Export Configuration")
                    {
                        Parameter param = att.GetParameter("data");
                        data = param.GetStringValue();
                        logger.Info("URDF export config \n " + data);
                    }
                }
              
            }
            if (!data.Equals(""))
            {
                logger.Info("Config data was not empty, deserializing");
                URDFExporter Exporter;

                XmlSerializer serializer = new XmlSerializer(typeof(URDFExporter));
                XmlTextReader textReader = new XmlTextReader(new StringReader(data));
                Exporter = (URDFExporter)serializer.Deserialize(textReader);
                textReader.Close();

                return Exporter;
            }
            return null;
        }

        public void FlyoutCallback()
        {
            FlyoutGroup flyGroup = iCmdMgr.GetFlyoutGroup(flyoutGroupID);
            flyGroup.RemoveAllCommandItems();

            flyGroup.AddCommandItem(System.DateTime.Now.ToLongTimeString(), "test", 0, "FlyoutCommandItem1", "FlyoutEnableCommandItem1");

        }
        public int FlyoutEnable()
        {
            return 1;
        }

        public void FlyoutCommandItem1()
        {
            iSwApp.SendMsgToUser("Flyout command 1");
        }

        public int FlyoutEnableCommandItem1()
        {
            return 1;
        }
        #endregion

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
                SwEventPtr.ActiveDocChangeNotify += new DSldWorksEvents_ActiveDocChangeNotifyEventHandler(OnDocChange);
                SwEventPtr.DocumentLoadNotify2 += new DSldWorksEvents_DocumentLoadNotify2EventHandler(OnDocLoad);
                SwEventPtr.FileNewNotify2 += new DSldWorksEvents_FileNewNotify2EventHandler(OnFileNew);
                SwEventPtr.ActiveModelDocChangeNotify += new DSldWorksEvents_ActiveModelDocChangeNotifyEventHandler(OnModelChange);
                SwEventPtr.FileOpenPostNotify += new DSldWorksEvents_FileOpenPostNotifyEventHandler(FileOpenPostNotify);
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
                SwEventPtr.ActiveDocChangeNotify -= new DSldWorksEvents_ActiveDocChangeNotifyEventHandler(OnDocChange);
                SwEventPtr.DocumentLoadNotify2 -= new DSldWorksEvents_DocumentLoadNotify2EventHandler(OnDocLoad);
                SwEventPtr.FileNewNotify2 -= new DSldWorksEvents_FileNewNotify2EventHandler(OnFileNew);
                SwEventPtr.ActiveModelDocChangeNotify -= new DSldWorksEvents_ActiveModelDocChangeNotifyEventHandler(OnModelChange);
                SwEventPtr.FileOpenPostNotify -= new DSldWorksEvents_FileOpenPostNotifyEventHandler(FileOpenPostNotify);
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
            ModelDoc2 modDoc = (ModelDoc2)iSwApp.GetFirstDocument();
            while (modDoc != null)
            {
                if (!openDocs.Contains(modDoc))
                {
                    AttachModelDocEventHandler(modDoc);
                }
                else if (openDocs.Contains(modDoc))
                {
                    bool connected = false;
                    DocumentEventHandler docHandler = (DocumentEventHandler)openDocs[modDoc];
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
                return false;

            DocumentEventHandler docHandler = null;

            if (!openDocs.Contains(modDoc))
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
                openDocs.Add(modDoc, docHandler);
            }
            return true;
        }

        public bool DetachModelEventHandler(ModelDoc2 modDoc)
        {
            DocumentEventHandler docHandler;
            docHandler = (DocumentEventHandler)openDocs[modDoc];
            openDocs.Remove(modDoc);
            modDoc = null;
            docHandler = null;
            return true;
        }

        public bool DetachEventHandlers()
        {
            DetachSwEvents();

            //Close events on all currently open docs
            DocumentEventHandler docHandler;
            int numKeys = openDocs.Count;
            object[] keys = new Object[numKeys];

            //Remove all document event handlers
            openDocs.Keys.CopyTo(keys, 0);
            foreach (ModelDoc2 key in keys)
            {
                docHandler = (DocumentEventHandler)openDocs[key];
                docHandler.DetachEventHandlers(); //This also removes the pair from the hash
                docHandler = null;
            }
            return true;
        }
        #endregion

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

        int FileOpenPostNotify(string FileName)
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




        #endregion


    }

}
