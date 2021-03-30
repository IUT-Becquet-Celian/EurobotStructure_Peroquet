using EventArgsLibrary;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Utilities;

namespace Positioning2WheelsNS
{
    public class Positioning2Wheels
    {
        int robotId;
        Location posRobotRefTerrain = new Location();
        double Fech = 50f;

        public Positioning2Wheels(int id)
        {
            robotId = id;
        }
        public void OnOdometryRobotSpeedReceived(object sender, PolarSpeedArgs e)
        {
            posRobotRefTerrain.Theta += (float)e.Vtheta / Fech;
            posRobotRefTerrain.X += (float)e.Vx / Fech * (float)Math.Cos(posRobotRefTerrain.Theta);
            posRobotRefTerrain.Y += (float)e.Vx / Fech * (float)Math.Sin(posRobotRefTerrain.Theta);
            posRobotRefTerrain.Vx = (float)e.Vx * (float)Math.Cos(posRobotRefTerrain.Theta);
            posRobotRefTerrain.Vy = (float)e.Vx * (float)Math.Sin(posRobotRefTerrain.Theta);

            //Console.WriteLine("posX : " + posX + "  posY : " + posY + "  Theta : " + Theta);
            OnCalculatedLocation(robotId, posRobotRefTerrain);
        }

        //Output events
        public event EventHandler<LocationArgs> OnCalculatedLocationEvent;
        public virtual void OnCalculatedLocation(int id, Location locationRefTerrain)
        {
            OnCalculatedLocationEvent?.Invoke(this, new LocationArgs { RobotId = id, Location = locationRefTerrain });
        }
    }
}
