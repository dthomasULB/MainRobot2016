/*
 * File:   Scheduler.c
 * Author: DenisT
 *
 * Created on 7 avril 2016, 21:59
 */

#include "Scheduler.h"
#include "dataTypes.h"
#include "basicActions.h"
#include "gestionPropulsion.h"
#include "Globals.h"
#include "sharp.h"
#include "uart.h"
#include "../Can/CanNetwork.h"
#include "../libdspic/CanDspic.h"
#include "../libdspic/clock.h"
#include "../libdspic/servo.h"
#include <math.h>
#include "BLE.h"
#include <libpic30.h>

infoRotbot petitRobot={2,{0,0,0},{0,0,1,0},0,100,0,0};
infoRotbot grandRobot={1,{0,0,0},{1,1,0,1},0,100,0,0};

actionType Porte[3]= {{{2800, 1750, 1800}, AUCUNE_ACTION,seDeplacer,A_FAIRE},{{2800, 1750, 1800}, AUCUNE_ACTION,pousser,A_FAIRE},{{2300, 1750, 1800}, AUCUNE_ACTION,seDeplacer,A_FAIRE}}; 
actionType Poisson[6]= {{{2500,150, 1800}, AUCUNE_ACTION,seDeplacer,A_FAIRE},{{2500, 150, 1800}, ACTIONNEUR,prendre,A_FAIRE},{{2100,150, 1800}, ACTIONNEUR_ACTIF,seDeplacer,A_FAIRE},{{2100, 150, 1800}, ACTIONNEUR_ACTIF,prendre,A_FAIRE},{{1700, 150, 1800}, AUCUNE_ACTION,seDeplacer,A_FAIRE},{{1700, 150, 1800}, ACTIONNEUR_RENTRER,prendre,A_FAIRE}};
actionType generAct[2]= {{{1000,1000, 1800}, AUCUNE_ACTION,gener,A_FAIRE},{{2800,1300, 1800}, AUCUNE_ACTION,gener,A_FAIRE}};
actionType SableDune[5] = {{{1950,1800, 1800}, AUCUNE_ACTION,seDeplacer,A_FAIRE},{{1950,1800, 1800}, ACTIONNEUR,prendre,A_FAIRE},{{1400,1800, 1800}, ACTIONNEUR_ACTIF,seDeplacer,A_FAIRE},{{1700,900, 1800}, AUCUNE_ACTION,seDeplacer,A_FAIRE},{{2100,900,1800}, ACTIONNEUR,seDeplacer,A_FAIRE}};
actionType SableDepart[3] = {{{1950,1800, 1800}, AUCUNE_ACTION,seDeplacer,A_FAIRE},{{1950,1800, 1800}, ACTIONNEUR,prendre,A_FAIRE},{{1400,1800, 1800}, AUCUNE_ACTION,seDeplacer,A_FAIRE}};
actionType SableDuneAlly[3] = {{{1950,1800, 1800}, AUCUNE_ACTION,seDeplacer,A_FAIRE},{{1950,1800, 1800}, ACTIONNEUR,prendre,A_FAIRE},{{1400,1800, 1800}, AUCUNE_ACTION,seDeplacer,A_FAIRE}};
// On compte � partir de 0 !!! (en nombre d'actions)
actionPossible ActionPossible[4] = {{5,0,0,&Poisson},{2,0,0,&Porte},{1,0,0,&generAct},{4,200000,0,&SableDune}};



actionType Scheduler(infoRotbot robot) {
      // Un des robots demandes une t�che � faire
    actionType TacheChoisie;
	int indiceAction = robot.Action;		//<! d�finit l'action en cours
    int indiceTache = robot.Tache;
    int oldIndiceTache = robot.Tache;
   
    float temps[4]={0,0,0,0};
    int tmp;
    int itache;
    int iaction;
    
    if (robot.OBST == 1) {
        ActionPossible[indiceTache].done = REPORTEE; // Ralentie � cause d'un obstac    
        //STOPED_OBS++;          // Si on a �t� arret� par un obstacle il faut recalculer
    }
    if (robot.ERROR == 3){
        ActionPossible[indiceTache].done = NO_TRAJECT;
    }
    
    if (indiceAction >= ActionPossible[indiceTache].numAction || robot.OBST){     // on a soit finit la t�che soit un obstacle
        for (itache=0; itache < 4;itache++){                                          // on parcourt toutes les t�ches de cette ann�e
             positionInteger fictivePos = robot.position;
            for (iaction=0; iaction < ActionPossible[itache].numAction;iaction++) {   // Chaque action d'une t�che pour calculer le temps estim� et faire un choix points/temps
                if(ActionPossible[itache].action[iaction].done == A_FAIRE){           // Si l'�tape est r�solue ou non
                    tmp=computeTime(ActionPossible[itache].action[iaction].dest,fictivePos);
                    if (ActionPossible[itache].action[iaction].methode == ACTIONNEUR || ActionPossible[itache].action[iaction].methode == ACTIONNEUR_ACTIF || ActionPossible[itache].action[iaction].methode == ACTIONNEUR_RENTRER){
                        tmp += 2000; // on compte 4 seconde par actionneur (ex: p�cher les poissons)
                    }    
                    temps[itache] += tmp;
                    fictivePos = ActionPossible[itache].action[iaction].dest;
                }
            }
            switch(ActionPossible[itache].done){
                case(A_FAIRE):
                    temps[itache] = (float) (ActionPossible[itache].point/temps[itache]);
                    if (temps[itache] < 1){
                        temps[itache]=1;            // on met au moins � un pour pas refaire une action d�j� faite
                    }
                    break;
                case(REPORTEE):
                    ActionPossible[indiceTache].done =A_FAIRE;
                    temps[itache]= (float) (ActionPossible[itache].point/(temps[itache]+3000)); // retard = +3 sec
                    break;
                case(FAITE):
                    temps[itache]= 0;
                    break;
                case(NO_TRAJECT):
                    ActionPossible[indiceTache].done =A_FAIRE;
                    temps[itache]= 0;
                    break;
                default:
                    break;
            }
    }
        indiceTache = find_maximun(temps,4); // deuxi�me arg la taille de temps
        if (oldIndiceTache != indiceTache || indiceAction == 100){            // Si on change on r�initialise les actions, si on change pas de t�che on 
            indiceAction = 0;
            if (ActionPossible[oldIndiceTache].done != FAITE){ // Si la t�che pr�c�dente n'a pas �t� finie.
                for (iaction=0; iaction < ActionPossible[oldIndiceTache].numAction;iaction++){
                    ActionPossible[oldIndiceTache].action[iaction].done = A_FAIRE;       // on remet toute actions d�j� FAITE � A_FAIRE
                }
            }
        }
        ActionPossible[indiceTache].done = FAITE;                 // On dit que la t�che est faite = Prise
    }
    else if (robot.ERROR !=1){                                         // Si on est en erreur on repeat
        ActionPossible[indiceTache].action[indiceAction].done = FAITE; // L'action pr�c�dente a �t� execut�e.
        indiceAction++;
    }       
    

    switch(robot.id){
        case 1:
            grandRobot.ERROR = 0;               // On "d�sactive" l'erreur
            grandRobot.Tache = indiceTache;
            grandRobot.Action = indiceAction;
            TacheChoisie= ActionPossible[indiceTache].action[indiceAction];     // On "envoie" au grand robot mais comme est d�j� dedans :p
            return(TacheChoisie);
        case 2:
            petitRobot.ERROR = 0;
            petitRobot.Tache = indiceTache;
            petitRobot.Action = indiceAction;
            radioSendChar('T');
            radioSendChar((char)indiceTache +'0');
            radioSendChar('A');
            radioSendChar((char)indiceAction +'0');
            radioSendChar('@');
            break;
            
        default:
            break;
    }
}
infoRotbot getGrandRobot(){
    grandRobot.position = propulsionGetPosition();
    return (grandRobot);
}

infoRotbot getPetitRobot(){
    // On met � jour les variables quand on demande de voir l'�tat petit robot
    petitRobot.position = getAllyPos();
    petitRobot.OBST = getOBSTAlly();
    return (petitRobot);
}

void setGrandRobotObs(int tmp){
    grandRobot.OBST = tmp;
}

void setGrandRobotERROR(int tmp){
    grandRobot.ERROR = tmp;
}

void setPetitRobotERROR(int tmp){
    petitRobot.ERROR = tmp;
}