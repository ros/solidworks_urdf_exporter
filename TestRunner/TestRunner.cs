using SolidWorks.Interop.sldworks;
using System;
using System.IO;
using System.Threading;
using Xunit.Abstractions;
using Xunit.Runners;

namespace TestRunner
{
    public class Program
    {
        // We use consoleLock because messages can arrive in parallel, so we want to make sure we get
        // consistent console output.
        static readonly object consoleLock = new object();

        // Use an event to know when we're done
        static readonly ManualResetEvent finished = new ManualResetEvent(false);

        // Start out assuming success; we'll set this to 1 if we get a failed test
        static int result = 0;

        static string TestNameFilter = "";

        public static int Main(string[] args)
        {
            string solutionDir =
                Path.GetDirectoryName( // sw2urdf
                Path.GetDirectoryName( // TestRunner
                Path.GetDirectoryName( // bin
                Path.GetDirectoryName( // x64
                Path.GetDirectoryName( // net452
                    AppDomain.CurrentDomain.BaseDirectory // Debug
                )))));

            string testAssembly = Path.Combine(solutionDir, "SW2URDF\\bin\\x64\\Debug\\SW2URDF.dll");
            string typeName = null;

            using (var runner = AssemblyRunner.WithAppDomain(testAssembly))
            {
                if (args.Length > 0)
                {
                    TestNameFilter = args[0];
                    runner.TestCaseFilter += FilterByClass;
                }
                runner.OnDiscoveryComplete = OnDiscoveryComplete;
                runner.OnExecutionComplete = OnExecutionComplete;
                runner.OnTestFailed = OnTestFailed;
                runner.OnTestSkipped = OnTestSkipped;

                Console.WriteLine("Discovering...");
                runner.Start(typeName);

                finished.WaitOne();
                finished.Dispose();
                return result;
            }
        }

        public static bool FilterByClass(ITestCase testCase)
        {
            if (testCase.DisplayName.Contains(TestNameFilter))
            {
                return true;
            }
            return false;
        }

        static void OnDiscoveryComplete(DiscoveryCompleteInfo info)
        {
            lock (consoleLock)
                Console.WriteLine($"Running {info.TestCasesToRun} of {info.TestCasesDiscovered} tests...");
        }

        static void OnExecutionComplete(ExecutionCompleteInfo info)
        {
            lock (consoleLock)
                Console.WriteLine(
                    $"Finished: {info.TotalTests} tests in" + 
                    $"{Math.Round(info.ExecutionTime, 3)}s " + 
                    $"({info.TestsFailed} failed, " + 
                    $"{info.TestsSkipped} skipped)");

            finished.Set();
        }

        static void OnTestFailed(TestFailedInfo info)
        {
            lock (consoleLock)
            {
                Console.ForegroundColor = ConsoleColor.Red;

                Console.WriteLine("[FAIL] {0}: {1}", info.TestDisplayName, info.ExceptionMessage);
                if (info.ExceptionStackTrace != null)
                    Console.WriteLine(info.ExceptionStackTrace);

                Console.ResetColor();
            }

            result = 1;
        }

        static void OnTestSkipped(TestSkippedInfo info)
        {
            lock (consoleLock)
            {
                Console.ForegroundColor = ConsoleColor.Yellow;
                Console.WriteLine("[SKIP] {0}: {1}", info.TestDisplayName, info.SkipReason);
                Console.ResetColor();
            }
        }
    }
}
