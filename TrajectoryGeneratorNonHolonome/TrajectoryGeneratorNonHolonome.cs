﻿using EventArgsLibrary;
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
        AvanceOuRecule,
        Recule,
        FreinageUrgence,
    }
    
    public class TrajectoryGeneratorNonHolonome
    {
        float Fech = 50f;

        int robotId;

        PointD destination = new PointD(0,1);
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
            PID_Position_Lineaire = new AsservissementPID(20, 0, 0.2, 4, 4, 4);
            PID_Position_Angulaire = new AsservissementPID(15, 0, 0.5, Math.PI/2, Math.PI / 2, Math.PI / 2);
        }

        public void InitRobotPosition(double x, double y, double theta)
        {
            Location old_currectLocation = currentLocationRefTerrain;
            currentLocationRefTerrain = new Location(x, y, theta, 0, 0, 0);
            wayPointLocation = new Location(x, y, theta, 0, 0, 0);
            ghostLocationRefTerrain = new Location(x, y, theta, 0, 0, 0);
            PIDPositionReset();
        }

        public void SetDestination(double x, double y)
        {
            destination = new PointD(x, y);
            trajectoryState = TrajectoryState.FreinageUrgence;
        }


        public void Recule(double distance)
        {

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
                    vitesseLineaireGhost = 0;
                    break;
                case TrajectoryState.FreinageUrgence:

                    if (Math.Abs(vitesseLineaireGhost) > 0.01)
                    {
                        vitesseLineaireGhost -= accelerationLineaire * 1 / Fech;
                    }
                    
                    else trajectoryState = TrajectoryState.Rotation;

                    ghostLocationRefTerrain.X += vitesseLineaireGhost / Fech * Math.Cos(ghostLocationRefTerrain.Theta);
                    ghostLocationRefTerrain.Y += vitesseLineaireGhost / Fech * Math.Sin(ghostLocationRefTerrain.Theta);

                    break;
                case TrajectoryState.Rotation:
                    {
                        /// On calcule dans un premier temps la distance d'arret du ghost
                        double angleArretGhost = ghostLocationRefTerrain.Vtheta * ghostLocationRefTerrain.Vtheta / (2 * accelerationAngulaire);
                        /// Puis on calcule l'angle cible
                        double angleCible = Math.Atan2(destination.Y - ghostLocationRefTerrain.Y, destination.X - ghostLocationRefTerrain.X);
                        /// puis on calcule l'angle restant à parcourir,
                        double angleRestant = angleCible - Toolbox.ModuloByAngle(angleCible, ghostLocationRefTerrain.Theta);

                       //angleRestant = Toolbox.ModuloPiAngleRadian(angleRestant); pour test de recule

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

                        /// Intégration de la vitesse pour avoir l'angle theta
                        ghostLocationRefTerrain.Theta += ghostLocationRefTerrain.Vtheta / Fech;

                        /// On regarde si on a atteint l'angle de destination ou pas
                        if (Math.Abs(angleRestant)<Toolbox.DegToRad(0.2))
                            trajectoryState = TrajectoryState.AvanceOuRecule;

                    }
                    
                    break;
                case TrajectoryState.AvanceOuRecule:
                    {
                        PointD positionGhost = new PointD(ghostLocationRefTerrain.X, ghostLocationRefTerrain.Y);
                        PointD pointDroite = new PointD(ghostLocationRefTerrain.X + Math.Cos(ghostLocationRefTerrain.Theta), ghostLocationRefTerrain.Y + Math.Sin(ghostLocationRefTerrain.Theta));
                        ///On détermine le point cible projeté
                        pointCible = Toolbox.ProjectionPointToLine(destination, positionGhost, pointDroite);
                        ///On calcule dans un premier temps la distance d'arret du ghost
                        double distanceArretGhost = vitesseLineaireGhost * vitesseLineaireGhost / (2 * accelerationLineaire);
                        ///Puis on calcule la distance cible
                        double distanceRestante = Math.Sqrt(Math.Pow(pointCible.Y - ghostLocationRefTerrain.Y, 2) + Math.Pow(pointCible.X - ghostLocationRefTerrain.X, 2));
                        ///Puis on calcul l'angle entre le robot et le pt Cible
                        double angleCible = Math.Atan2(destination.Y - ghostLocationRefTerrain.Y, destination.X - ghostLocationRefTerrain.X);
                        double angleRobotCible = Toolbox.ModuloByAngle(ghostLocationRefTerrain.Theta, angleCible) - ghostLocationRefTerrain.Theta;

                        // ------------- Dépassement -------------

                        bool cibleDevant = true;
                        if (Math.Abs(angleRobotCible) > Math.PI / 2)
                            cibleDevant = false;

                        double coeffMajoration = 
                            2;

                        if(cibleDevant)
                        {
                            //La vitesse doit impérativement tendre à être positive !
                            if (vitesseLineaireGhost < 0)
                            {
                                //on freine en reculant pour revenir à une vitesse positive
                                vitesseLineaireGhost += accelerationLineaire * 1 / Fech;
                            }
                            else
                            {
                                if (distanceRestante > distanceArretGhost * coeffMajoration)
                                {
                                    if (Math.Abs(ghostLocationRefTerrain.Vlin) < vitesseLineaireMax)
                                        vitesseLineaireGhost += accelerationLineaire * 1 / Fech;
                                    else
                                        ;//rien du tout, on est à Vmax
                                }
                                else
                                    vitesseLineaireGhost -= accelerationLineaire * 1 / Fech;
                            }
                        }
                        else
                        {
                            //La vitesse doit impérativement tendre à être négative
                            if(vitesseLineaireGhost > 0)
                            {
                                //On frein en avancant pour revenir à une vitesse négative
                                vitesseLineaireGhost -= accelerationLineaire * 1 / Fech;
                            }
                            else
                            {
                                if (distanceRestante > distanceArretGhost * coeffMajoration)
                                {
                                    if (Math.Abs(vitesseLineaireGhost) < vitesseLineaireMax)
                                        vitesseLineaireGhost -= accelerationLineaire * 1 / Fech;
                                    else
                                        ;//rien du tout, on est à Vmax
                                }
                                else
                                    vitesseLineaireGhost += accelerationLineaire * 1 / Fech;
                            }
                        }
                        /*
                        /// On regarde si on peut accélérer ou si il faut freiner ou rester à vitesse constante
                        if (distanceArretGhost < Math.Abs(distanceRestante))
                        {
                            if (Math.Abs(vitesseLineaireGhost) < vitesseLineaireMax)
                            {
                                /// On peut accélérer
                                if(Math.Abs(angleRobotCible)< Math.PI/2)
                                    vitesseLineaireGhost += accelerationLineaire * 1 / Fech;
                                else
                                    vitesseLineaireGhost -= accelerationLineaire * 1 / Fech;
                            }
                            else
                            {
                                ///Rien, on reste à la même vitesse
                            }
                        }
                        else
                        {
                            /// On doit freiner
                            if (Math.Abs(angleRobotCible) < Math.PI / 2)
                                vitesseLineaireGhost -= accelerationLineaire * 1 / Fech;
                            else
                                vitesseLineaireGhost += accelerationLineaire * 1 / Fech;
                        }
                        */

                        ghostLocationRefTerrain.X += vitesseLineaireGhost / Fech * Math.Cos(ghostLocationRefTerrain.Theta);
                        ghostLocationRefTerrain.Y += vitesseLineaireGhost / Fech * Math.Sin(ghostLocationRefTerrain.Theta);

                        ///On regarde si on a atteint le point
                        if (distanceRestante < 0.0002)
                        {
                            trajectoryState = TrajectoryState.Idle;
                            OnDestinationReached();
                        }
                            
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
            double erreurLineaire = 0;
            //A remplir
            double vLineaireRobot=0, vAngulaireRobot=0;

            double erreurTheta = ghostLocationRefTerrain.Theta - Toolbox.ModuloByAngle(ghostLocationRefTerrain.Theta, currentLocationRefTerrain.Theta); //calcul de l'erreur theta
            vAngulaireRobot = PID_Position_Angulaire.CalculatePDoutput(erreurTheta, 1 / Fech); //calcul du PD angulaire

            PointD positionRobot = new PointD(currentLocationRefTerrain.X, currentLocationRefTerrain.Y);
            PointD pointRobot2 = new PointD(currentLocationRefTerrain.X + Math.Cos(currentLocationRefTerrain.Theta), currentLocationRefTerrain.Y + Math.Sin(currentLocationRefTerrain.Theta));
            PointD positionGhost = new PointD(ghostLocationRefTerrain.X, ghostLocationRefTerrain.Y);
            ///On détermine la distance entre le projeté du ghost sur l'axe du robot et le robot
            PointD projectionGhost= Toolbox.ProjectionPointToLine(positionGhost, positionRobot, pointRobot2);

            double angleDirectionRobot = Math.Atan2(pointRobot2.Y - currentLocationRefTerrain.Y, pointRobot2.X - currentLocationRefTerrain.X);
            double angleRobotProjeteGhost = Math.Atan2(projectionGhost.Y - currentLocationRefTerrain.Y, projectionGhost.X - currentLocationRefTerrain.X);
            double anglePolarisant = angleDirectionRobot - Toolbox.ModuloByAngle(angleDirectionRobot, angleRobotProjeteGhost);
            if (Math.Abs(anglePolarisant) > Math.PI / 2)
            {
                //double erreurLin = Math.Sqrt(Math.Pow((pointCible.X - currentLocationRefTerrain.X), 2) + Math.Pow((pointCible.Y - currentLocationRefTerrain.Y), 2));
                erreurLineaire = -Toolbox.Distance(projectionGhost, positionRobot);
            }
            else  erreurLineaire = Toolbox.Distance(projectionGhost, positionRobot);




            vLineaireRobot = PID_Position_Lineaire.CalculatePDoutput(erreurLineaire, 1 / Fech);

            //Si tout s'est bien passé, on envoie les vitesses consigne.
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

        /*************************************** Incoming Events ************************************/

        public void OnDestinationReceived(object sender, PositionArgs e)
        {
            SetDestination(e.X, e.Y);
            //throw new NotImplementedException();
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

        public event EventHandler<LocationArgs> OnNewLocationReceivedEvent;
        public virtual void OnNewLocationReceived(int id, Location loc)
        {
            OnNewLocationReceivedEvent?.Invoke(this, new LocationArgs { RobotId = id, Location = loc });
        }

        public event EventHandler<EventArgs> OnDestinationReachedEvent;
        public virtual void OnDestinationReached()
        {
            OnDestinationReachedEvent?.Invoke(this, new EventArgs());
        }
    }
}