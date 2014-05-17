using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using ABB.Robotics;
using ABB.Robotics.Controllers;
using ABB.Robotics.Controllers.Discovery;
using ABB.Robotics.Controllers.RapidDomain;
using Task = ABB.Robotics.Controllers.RapidDomain.Task;

namespace Dynamo_ABB
{

    public class DynamoABB
    {
        private NetworkScanner scanner = null;
        private Controller controller = null;
        private Task[] tasks = null;
        private NetworkWatcher networkwatcher = null;

        public static List<Controller> FindControllersAtIp(string address)
        {
            var controllers = new List<Controller>();


            return controllers;
        }
    }
}
