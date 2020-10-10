using SolidWorks.Interop.sldworks;
using System;

namespace SW2URDF.Test
{
    /// <summary>
    /// TestFixture which gets passed to each Test Class. For now it just provides 
    /// the reference to the SolidWorks app.
    /// </summary>
    public class SWTestFixture : IDisposable
    {
        public static bool Initialized = false;
        public static SldWorks SwApp;

        public static void Initialize()
        {
            if (!Initialized)
            {
                SwApp = (SldWorks)Activator.CreateInstance(Type.GetTypeFromProgID("SldWorks.Application"));
                SwApp.Visible = true;
                Initialized = true;
            }
        }

        public void Dispose()
        {
            Dispose(true);
            GC.SuppressFinalize(this);
        }

        protected virtual void Dispose(bool disposing)
        {

        }
    }
}
