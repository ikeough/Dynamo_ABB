using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Text;
using ABB.Robotics.Controllers;
using ABB.Robotics.Controllers.Discovery;
using ABB.Robotics.Controllers.RapidDomain;
using Autodesk.DesignScript.Geometry;

namespace Dynamo_ABB
{
    public class DynamoABB
    {
        /// <summary>
        /// Set a rapid data object's value in the specified module.
        /// </summary>
        /// <param name="moduleName">The name of the module.</param>
        /// <param name="variableName">The name of the variable.</param>
        /// <param name="variableValue">The value of the variable.</param>
        public static void RobotTargetAtVariableFromString(string moduleName, string variableName, string variableValue)
        {
            try
            {
                var scanner = new NetworkScanner();
                scanner.Scan();

                ControllerInfoCollection controllers = scanner.Controllers;
                using (var controller = ControllerFactory.CreateFrom(controllers[0]))
                {
                    controller.Logon(UserInfo.DefaultUser);
                    using (Mastership.Request(controller.Rapid))
                    {
                        if (controller.OperatingMode == ControllerOperatingMode.Auto)
                        {
                            using (var task = controller.Rapid.GetTask("T_ROB1"))
                            {
                                var target = new RobTarget();
                                using (var rapidData = task.GetRapidData(moduleName, variableName))
                                {
                                    if (rapidData.Value is RobTarget)
                                    {
                                        target.FillFromString2(variableValue);
                                        rapidData.Value = target;
                                    }
                                }
 
                                var result = task.Start(true);
                                Debug.WriteLine(result.ToString());

                                task.ResetProgramPointer();
                            }
                        }
                        else
                        {
                            Debug.WriteLine("Automatic mode is required to start execution from a remote client.");
                        }
                    }
                    controller.Logoff();
                }
                
            }
            catch (Exception ex)
            {
                Debug.WriteLine(ex.Message);
                Debug.WriteLine(ex.StackTrace);
            }
        }

        /// <summary>
        /// Create a Robot target from a point.
        /// </summary>
        /// <param name="p">The point.</param>
        /// <returns></returns>
        public static RobTarget TargetAtPoint(Point point)
        {
            var target = new RobTarget();
            target.FillFromString2(string.Format("[[{0},{1},{2}],[0.000000,0.000000,-1.000000,0.000000082],[0,-1,0,1],[9E9,9E9,9E9,9E9,9E9,9E9]];", point.X, point.Y, point.Z));
            return target;
        }

        /// <summary>
        /// Write a Rapid file from a set of targets.
        /// </summary>
        /// <param name="targets">A list of targets.</param>
        /// <param name="filePath">A file path.</param>
        /// <returns></returns>
        public static string WriteRapidFile(List<RobTarget> targets, string filePath)
        {
            var targetIds = new List<string>();

            var pathBuilder = new StringBuilder();
            var counter = 0;
            foreach (var target in targets)
            {
                var targetId = "b"+counter;
                targetIds.Add(targetId);
                pathBuilder.AppendLine(string.Format("\tCONST robtarget {0}:={1};", targetId, target));
                counter++;
            }

            var moveBuilder = new StringBuilder();
            foreach (var id in targetIds)
            {
                moveBuilder.AppendLine(string.Format("\t\tMoveL {0},v100,z1,tPen\\WObj:=WobjPad;", id));
            }

            using (var tw = new StreamWriter(filePath, false))
            {
                var rapid = string.Format("MODULE MainModule\n"+
                                            "\tPERS tooldata tPen:=[TRUE,[[-50.828842163,-0.015067339,170.179992676],[1,0,0,0]],[1,[-46.200036966,0.000035189,42.434212285],[1,0,0,0],0,0,0]];\n"+
                                            "\tTASK PERS wobjdata WobjPad:=[FALSE,TRUE," + @"""""" + ",[[-295.62617401,17.579148369,72.979639055],[1,0,0,0]],[[0,0,0],[1,0,0,0]]];\n"+ 
                                            "\t! targets for curve\n"+
                                            "{0}\n"+
                                            "\t! Main routine\n"+
                                            "\tPROC main()\n"+
                                            "\n"+
                                            "\t\tConfL \\Off;\n"+
                                            "\t\trStart;\n"+
                                            "\t\tRETURN;\n"+
                                            "\tENDPROC\n"+
                                            "\tPROC rStart()\n"+
                                            "{1}\n"+
                                            "\t\tRETURN;\n"+
                                            "\tENDPROC\n"+
                                            "ENDMODULE\n",
                pathBuilder.ToString(), moveBuilder.ToString());

                tw.Write(rapid);
                tw.Flush();
            }

            var prfPath = Path.Combine(Path.GetDirectoryName(filePath), "Dynamo.prg");
            using (var tw = new StreamWriter(prfPath, false))
            {
                var prf = "<?xml version=\"1.0\" encoding=\"ISO-8859-1\"?>\n"+
                            "<Program entry=\"main\">\n"+
	                        "<Module>MainModule.mod</Module>\n"+
                            "</Program>\n";

                tw.Write(prf);
                tw.Flush();
            }

            return filePath;
        }

        /// <summary>
        /// Write a Rapid program to your controller and execute it.
        /// This method automatically generates a Dynamo.prg file in the same
        /// directory as the file path argument.
        /// </summary>
        /// <param name="filePath">The path of the file to upload on the local file system.</param>
        /// <returns></returns>
        public static bool WriteProgramToController(string filePath)
        {
            if (!File.Exists(filePath))
            {
                throw new Exception("File could not be found.");
            }

            var scanner = new NetworkScanner();
            scanner.Scan();

            ControllerInfoCollection controllers = scanner.Controllers;
            using (var controller = ControllerFactory.CreateFrom(controllers[0]))
            {
                controller.Logon(UserInfo.DefaultUser);
                using (Mastership.Request(controller.Rapid))
                {
                    if (controller.OperatingMode == ControllerOperatingMode.Auto)
                    {
                        var fileName = Path.GetFileName(filePath);

                        try
                        {
                            var remoteDir = controller.FileSystem.RemoteDirectory.Replace('/', '\\');
                            var localDir = Path.GetDirectoryName(filePath);
                            controller.FileSystem.LocalDirectory = localDir;

                            //write the mod file
                            controller.FileSystem.PutFile(fileName, true);
                            
                            //write the prf file
                            controller.FileSystem.PutFile("Dynamo.prg", true);

                        }
                        catch (Exception ex)
                        {
                            Debug.WriteLine(ex.Message);
                            return false;
                        }
                        
                        using (var task = controller.Rapid.GetTask("T_ROB1"))
                        {
                            // read data from file
                            task.LoadProgramFromFile("Dynamo.prg", RapidLoadMode.Replace);
                            task.LoadModuleFromFile(fileName, RapidLoadMode.Add);
                            task.ResetProgramPointer();
                            var result = task.Start(true);
                            Debug.WriteLine(result.ToString());
                        }
                    }
                    else
                    {
                        Debug.WriteLine("Automatic mode is required to start execution from a remote client.");
                    }
                }
                controller.Logoff();
            }

            return true;
        }
    }
}
