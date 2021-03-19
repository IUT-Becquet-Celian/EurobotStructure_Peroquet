using EventArgsLibrary;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Utilities;

namespace TrajectoryGeneratorNonHolonomeNS
{
    public enum TrajectoryState
    {
        Idle,
        Rotation,
        Avance,
        Recule,
    }
    
    public class TrajectoryGeneratorNonHolonome
    {
        float Fech = 50f;

        int robotId;

        PointD destination = new PointD(3,2);
        PointD pointCible = new PointD(0,0);
        
       

        double samplingFreq;

        Location currentLocationRefTerrain;
        Location wayPointLocation;
        Location ghostLocationRefTerrain;

        double accelLineaire, accelAngulaire;
        double vitesseLineaireMax, vitesseAngulaireMax;

        double accelerationAngulaire = 1;        
        double accelerationLineaire = 1;
        double vitesseLineaireGhost;
        
        AsservissementPID PID_Position_Lineaire;
        AsservissementPID PID_Position_Angulaire;

        TrajectoryState trajectoryState = TrajectoryState.Rotation;

        public TrajectoryGeneratorNonHolonome(int id)
        {
            robotId = id;
            InitRobotPosition(0, 0, 0);
            InitPositionPID();

            //Initialisation des vitesse et accélérations souhaitées
            accelLineaire = 0.5; //en m.s-2
            accelAngulaire = 0.5 * Math.PI * 1.0; //en rad.s-2

            vitesseLineaireMax = 1; //en m.s-1               
            vitesseAngulaireMax = 1 * Math.PI * 1.0; //en rad.s-1
        }

        void InitPositionPID()
        {
            PID_Position_Lineaire = new AsservissementPID(20.0, 10.0, 0, 100, 100, 1);
            PID_Position_Angulaire = new AsservissementPID(20.0, 10.0, 0, 5 * Math.PI, 5 * Math.PI, Math.PI);
        }

        public void InitRobotPosition(double x, double y, double theta)
        {
            Location old_currectLocation = currentLocationRefTerrain;
            currentLocationRefTerrain = new Location(x, y, theta, 0, 0, 0);
            wayPointLocation = new Location(x, y, theta, 0, 0, 0);
            ghostLocationRefTerrain = new Location(x, y, theta, 0, 0, 0);
            PIDPositionReset();
        }

        public void OnPhysicalPositionReceived(object sender, LocationArgs e)
        {
            if (robotId == e.RobotId)
            {
                currentLocationRefTerrain = e.Location;
                CalculateGhostPosition();
                PIDPosition();
            }
        }

       

        void CalculateGhostPosition()
        {
            switch(trajectoryState)
            {
                case TrajectoryState.Idle:
                    
                    break;
                case TrajectoryState.Rotation:
                    {
                        /// On calcule dans un premier temps la distance d'arret du ghost
                        double angleArretGhost = ghostLocationRefTerrain.Vtheta * ghostLocationRefTerrain.Vtheta / (2 * accelerationAngulaire);
                        /// Puis on calcule l'angle cible
                        double angleCible = Math.Atan2(destination.Y - ghostLocationRefTerrain.Y, destination.X - ghostLocationRefTerrain.X);
                        /// puis on calcule l'angle restant à parcourir,
                        double angleRestant = angleCible - Toolbox.ModuloByAngle(angleCible, ghostLocationRefTerrain.Theta);

                        /// On regarde si on peut accélérer ou si il faut freiner ou rester à vitesse constante
                        if (angleArretGhost < Math.Abs(angleRestant))
                        {
                            if (Math.Abs(ghostLocationRefTerrain.Vtheta) < vitesseAngulaireMax)
                            {
                                /// On peut accélérer
                                if (angleRestant > 0)
                                    ghostLocationRefTerrain.Vtheta += accelerationAngulaire * 1 / Fech;
                                else
                                    ghostLocationRefTerrain.Vtheta -= accelerationAngulaire * 1 / Fech;
                            }
                            else
                            {
                                ///Rien, on reste à la même vitesse
                            }
                        }
                        else
                        {
                            /// On doit freiner
                            if (angleRestant > 0)
                                ghostLocationRefTerrain.Vtheta -= accelerationAngulaire * 1 / Fech;
                            else
                                ghostLocationRefTerrain.Vtheta += accelerationAngulaire * 1 / Fech;
                        }

                        ghostLocationRefTerrain.Theta += ghostLocationRefTerrain.Vtheta / Fech;

                        ///On regarde si on a atteint l'angle de destination ou pas
                        if (Math.Abs(angleRestant)<Toolbox.DegToRad(0.2))
                            trajectoryState = TrajectoryState.Avance;

                    }
                    
                    break;
                case TrajectoryState.Avance:
                    {
                        PointD positionGhost = new PointD(ghostLocationRefTerrain.X, ghostLocationRefTerrain.Y);
                        PointD pointDroite = new PointD(ghostLocationRefTerrain.X + Math.Cos(ghostLocationRefTerrain.Theta), ghostLocationRefTerrain.Y + Math.Sin(ghostLocationRefTerrain.Theta));
                        ///On détermine le point cible projeté
                        pointCible = Toolbox.ProjectionPointToLine(destination, positionGhost, pointDroite);
                        ///On calcule dans un premier temps la distance d'arret du ghost
                        double distanceArretGhost = vitesseLineaireGhost * vitesseLineaireGhost / (2 * accelerationLineaire);
                        ///Puis on calcule la distance cible
                        double distanceRestante = Math.Sqrt(Math.Pow(pointCible.Y - ghostLocationRefTerrain.Y, 2) + Math.Pow(pointCible.X - ghostLocationRefTerrain.X, 2));
                       
                        /// On regarde si on peut accélérer ou si il faut freiner ou rester à vitesse constante
                        if (distanceArretGhost < Math.Abs(distanceRestante))
                        {
                            if (Math.Abs(vitesseLineaireGhost) < vitesseLineaireMax)
                            {
                                /// On peut accélérer
                                vitesseLineaireGhost += accelerationLineaire * 1 / Fech;
                            }
                            else
                            {
                                ///Rien, on reste à la même vitesse
                            }
                        }
                        else
                        {
                            /// On doit freiner
                            vitesseLineaireGhost -= accelerationAngulaire * 1 / Fech;
                        }

                        ghostLocationRefTerrain.X += vitesseLineaireGhost / Fech * Math.Cos(ghostLocationRefTerrain.Theta);
                        ghostLocationRefTerrain.Y += vitesseLineaireGhost / Fech * Math.Sin(ghostLocationRefTerrain.Theta);

                        ///On regarde si on a atteint le point
                        if (distanceRestante < 0.002)
                            trajectoryState = TrajectoryState.Idle;
                    }
                    break;
                default:
                    break;
            }

            //On renvoie la position du ghost pour affichage
            OnGhostLocation(robotId, ghostLocationRefTerrain);
            
        }

        void PIDPosition()
        {
            //A remplir
            double vLineaireRobot=0, vAngulaireRobot=0;


            //Si tout c'est bien passé, on envoie les vitesses consigne.
            OnSpeedConsigneToRobot(robotId, (float)vLineaireRobot, (float)vAngulaireRobot);
        }

        void PIDPositionReset()
        {
            if (PID_Position_Angulaire != null && PID_Position_Lineaire != null)
            {
                PID_Position_Lineaire.ResetPID(0);
                PID_Position_Angulaire.ResetPID(0);
            }
        }

        /*************************************** Outgoing Events ************************************/

        public event EventHandler<LocationArgs> OnGhostLocationEvent;
        public virtual void OnGhostLocation(int id, Location loc)
        {
            var handler = OnGhostLocationEvent;
            if (handler != null)
            {
                handler(this, new LocationArgs { RobotId = id, Location = loc });
            }
        }

        public event EventHandler<PolarSpeedArgs> OnSpeedConsigneEvent;
        public virtual void OnSpeedConsigneToRobot(int id, float vLineaire, float vAngulaire)
        {
            var handler = OnSpeedConsigneEvent;
            if (handler != null)
            {
                handler(this, new PolarSpeedArgs { RobotId = id, Vx = vLineaire, Vy = 0, Vtheta = vAngulaire});
            }
        }
    }
}
